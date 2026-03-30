use std::{
    collections::HashMap,
    ffi::CString,
    ptr,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicBool, AtomicU32, Ordering},
        mpsc::SyncSender,
    },
};

use eyre::{Context, Result, bail};
use livox_sdk2_sys::{
    DisableLivoxSdkConsoleLogger, EnableLivoxLidarImuData, LivoxLidarAsyncControlResponse,
    LivoxLidarEthernetPacket, LivoxLidarInfo, LivoxLidarSdkInit, LivoxLidarSdkUninit, SDK2_LINKED,
    SetLivoxLidarImuDataCallback, SetLivoxLidarInfoChangeCallback, SetLivoxLidarPointCloudCallBack,
    SetLivoxLidarWorkMode, handle_to_ipv4, kLivoxLidarNormal, kLivoxLidarStatusSuccess,
    kLivoxLidarTypeMid360, livox_status,
};

use crate::packet_parser::{ParsedMessage, TimeSyncTracker, parse_packet_from_ffi};

#[derive(Debug)]
pub enum DriverMessage {
    Parsed(ParsedMessage),
    StopRequested,
    DoraClosed,
}

pub struct SdkRuntime {
    _config_path: CString,
}

struct CallbackState {
    tx: SyncSender<DriverMessage>,
    selected_handle: AtomicU32,
    warned_extra_handle: AtomicBool,
    warned_queue_full: AtomicBool,
    warned_parse_error: AtomicBool,
    tracker: Mutex<HashMap<u32, TimeSyncTracker>>,
    imu_frame_id: String,
}

static CALLBACK_STATE: OnceLock<Mutex<Option<Arc<CallbackState>>>> = OnceLock::new();

impl SdkRuntime {
    pub fn start(
        tx: SyncSender<DriverMessage>,
        config_path: &str,
        imu_frame_id: String,
    ) -> Result<Self> {
        if !SDK2_LINKED {
            bail!(
                "Livox-SDK2 shared library is not available; install it or set LIVOX_SDK2_LIB_DIR/LIVOX_SDK2_INCLUDE_DIR"
            );
        }

        let config_path =
            CString::new(config_path).context("SDK2 config path contains NUL byte")?;
        let state = Arc::new(CallbackState {
            tx,
            selected_handle: AtomicU32::new(0),
            warned_extra_handle: AtomicBool::new(false),
            warned_queue_full: AtomicBool::new(false),
            warned_parse_error: AtomicBool::new(false),
            tracker: Mutex::new(HashMap::new()),
            imu_frame_id,
        });
        install_callback_state(state.clone())?;

        let initialized =
            unsafe { LivoxLidarSdkInit(config_path.as_ptr(), ptr::null(), ptr::null()) };
        if !initialized {
            clear_callback_state();
            bail!(
                "failed to initialize Livox-SDK2 with config path {}",
                config_path.to_string_lossy()
            );
        }

        unsafe {
            DisableLivoxSdkConsoleLogger();
            SetLivoxLidarPointCloudCallBack(Some(point_cloud_callback), ptr::null_mut());
            SetLivoxLidarImuDataCallback(Some(imu_callback), ptr::null_mut());
            SetLivoxLidarInfoChangeCallback(Some(info_change_callback), ptr::null_mut());
        }

        Ok(Self {
            _config_path: config_path,
        })
    }
}

impl Drop for SdkRuntime {
    fn drop(&mut self) {
        clear_callback_state();
        unsafe { LivoxLidarSdkUninit() };
    }
}

fn install_callback_state(state: Arc<CallbackState>) -> Result<()> {
    let lock = CALLBACK_STATE.get_or_init(|| Mutex::new(None));
    let mut guard = lock.lock().unwrap();
    if guard.is_some() {
        bail!("Livox-SDK2 runtime is already initialized");
    }
    *guard = Some(state);
    Ok(())
}

fn clear_callback_state() {
    if let Some(lock) = CALLBACK_STATE.get() {
        let mut guard = lock.lock().unwrap();
        *guard = None;
    }
}

fn callback_state() -> Option<Arc<CallbackState>> {
    CALLBACK_STATE
        .get()
        .and_then(|lock| lock.lock().ok().and_then(|state| state.clone()))
}

unsafe extern "C" fn point_cloud_callback(
    handle: u32,
    _dev_type: u8,
    data: *mut LivoxLidarEthernetPacket,
    _client_data: *mut std::ffi::c_void,
) {
    let Some(state) = callback_state() else {
        return;
    };
    if !should_accept_handle(&state, handle) {
        return;
    }

    let mut tracker_guard = state.tracker.lock().unwrap();
    let tracker = tracker_guard.entry(handle).or_default();
    let lidar_ip = handle_to_ipv4(handle);
    match unsafe {
        parse_packet_from_ffi(tracker, lidar_ip, &state.imu_frame_id, data.cast_const())
    } {
        Ok(Some(ParsedMessage::PointsBatch(batch))) => send_message(
            &state,
            DriverMessage::Parsed(ParsedMessage::PointsBatch(batch)),
        ),
        Ok(Some(ParsedMessage::Imu(_))) => {}
        Ok(None) => {}
        Err(err) => warn_parse_error(
            &state,
            &format!("failed to parse point cloud packet: {err}"),
        ),
    }
}

unsafe extern "C" fn imu_callback(
    handle: u32,
    _dev_type: u8,
    data: *mut LivoxLidarEthernetPacket,
    _client_data: *mut std::ffi::c_void,
) {
    let Some(state) = callback_state() else {
        return;
    };
    if !should_accept_handle(&state, handle) {
        return;
    }

    let mut tracker_guard = state.tracker.lock().unwrap();
    let tracker = tracker_guard.entry(handle).or_default();
    let lidar_ip = handle_to_ipv4(handle);
    match unsafe {
        parse_packet_from_ffi(tracker, lidar_ip, &state.imu_frame_id, data.cast_const())
    } {
        Ok(Some(ParsedMessage::Imu(imu))) => {
            send_message(&state, DriverMessage::Parsed(ParsedMessage::Imu(imu)))
        }
        Ok(Some(ParsedMessage::PointsBatch(_))) => {}
        Ok(None) => {}
        Err(err) => warn_parse_error(&state, &format!("failed to parse imu packet: {err}")),
    }
}

unsafe extern "C" fn info_change_callback(
    handle: u32,
    info: *const LivoxLidarInfo,
    _client_data: *mut std::ffi::c_void,
) {
    let Some(state) = callback_state() else {
        return;
    };
    if info.is_null() {
        return;
    }

    let dev_type = unsafe { ptr::addr_of!((*info).dev_type).read_unaligned() };
    if dev_type != kLivoxLidarTypeMid360 {
        eprintln!("Ignoring non MID-360 device type {dev_type} for handle {handle}");
        return;
    }

    let selected = state.selected_handle.load(Ordering::SeqCst);
    if selected == 0 {
        if state
            .selected_handle
            .compare_exchange(0, handle, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
        {
            unsafe {
                SetLivoxLidarWorkMode(
                    handle,
                    kLivoxLidarNormal,
                    Some(control_callback),
                    ptr::null_mut(),
                );
                EnableLivoxLidarImuData(handle, Some(control_callback), ptr::null_mut());
            }
        }
    } else if selected != handle && !state.warned_extra_handle.swap(true, Ordering::SeqCst) {
        eprintln!("Ignoring additional MID-360 handle {handle}; v1 supports only one lidar");
    }
}

unsafe extern "C" fn control_callback(
    status: livox_status,
    handle: u32,
    response: *mut LivoxLidarAsyncControlResponse,
    _client_data: *mut std::ffi::c_void,
) {
    if status == kLivoxLidarStatusSuccess {
        return;
    }

    if response.is_null() {
        eprintln!("Livox control callback failed for handle {handle} with status {status}");
        return;
    }

    let ret_code = unsafe { ptr::addr_of!((*response).ret_code).read_unaligned() };
    let error_key = unsafe { ptr::addr_of!((*response).error_key).read_unaligned() };
    eprintln!(
        "Livox control callback failed for handle {handle}: status={status} ret_code={ret_code} error_key={error_key}"
    );
}

fn should_accept_handle(state: &CallbackState, handle: u32) -> bool {
    state.selected_handle.load(Ordering::SeqCst) == handle
}

fn send_message(state: &CallbackState, message: DriverMessage) {
    if state.tx.try_send(message).is_err() && !state.warned_queue_full.swap(true, Ordering::SeqCst)
    {
        eprintln!("MID-360 driver output channel is full; dropping data");
    }
}

fn warn_parse_error(state: &CallbackState, msg: &str) {
    if !state.warned_parse_error.swap(true, Ordering::SeqCst) {
        eprintln!("{msg}");
    }
}

#[cfg(test)]
mod tests {
    use std::{
        collections::HashMap,
        net::Ipv4Addr,
        ptr,
        sync::{
            Arc, Mutex,
            atomic::{AtomicBool, AtomicU32},
            mpsc::sync_channel,
        },
    };

    use livox_sdk2_sys::{
        LivoxLidarCartesianHighRawPoint, LivoxLidarEthernetPacket, LivoxLidarInfo,
        ethernet_header_size, kLivoxLidarCartesianCoordinateHighData, kLivoxLidarTypeMid360,
    };
    use serial_test::serial;

    use crate::{
        packet_parser::ParsedMessage,
        sdk_runtime::{
            CallbackState, DriverMessage, callback_state, clear_callback_state,
            info_change_callback, install_callback_state, point_cloud_callback,
        },
    };

    fn build_packet() -> Vec<u8> {
        let header_len = ethernet_header_size();
        let payload = [LivoxLidarCartesianHighRawPoint {
            x: 1000,
            y: 2000,
            z: 3000,
            reflectivity: 42,
            tag: 0,
        }];
        let total_len = header_len + std::mem::size_of_val(&payload);
        let mut buffer = vec![0u8; total_len];
        buffer[1..3].copy_from_slice(&(total_len as u16).to_le_bytes());
        buffer[3..5].copy_from_slice(&(100u16).to_le_bytes());
        buffer[5..7].copy_from_slice(&(1u16).to_le_bytes());
        buffer[10] = kLivoxLidarCartesianCoordinateHighData;
        buffer[11] = 1;
        buffer[32..40].copy_from_slice(&1_000u64.to_le_bytes());
        unsafe {
            buffer[header_len..]
                .as_mut_ptr()
                .cast::<LivoxLidarCartesianHighRawPoint>()
                .copy_from_nonoverlapping(payload.as_ptr(), payload.len());
        }
        buffer
    }

    #[test]
    #[serial]
    fn callback_shim_pushes_parsed_message() {
        clear_callback_state();
        let (tx, rx) = sync_channel(8);
        let state = Arc::new(CallbackState {
            tx,
            selected_handle: AtomicU32::new(0),
            warned_extra_handle: AtomicBool::new(false),
            warned_queue_full: AtomicBool::new(false),
            warned_parse_error: AtomicBool::new(false),
            tracker: Mutex::new(HashMap::new()),
            imu_frame_id: "livox_imu".to_owned(),
        });
        install_callback_state(state).unwrap();

        let mut info = LivoxLidarInfo::default();
        info.dev_type = kLivoxLidarTypeMid360;
        unsafe { info_change_callback(0x6401_A8C0, ptr::addr_of!(info), ptr::null_mut()) };
        assert!(callback_state().is_some());

        let mut packet = build_packet();
        unsafe {
            point_cloud_callback(
                0x6401_A8C0,
                0,
                packet.as_mut_ptr().cast::<LivoxLidarEthernetPacket>(),
                ptr::null_mut(),
            )
        };

        let DriverMessage::Parsed(ParsedMessage::PointsBatch(batch)) = rx.recv().unwrap() else {
            panic!("expected parsed point batch");
        };
        assert_eq!(batch.lidar_ip, Ipv4Addr::new(192, 168, 1, 100).to_string());
        assert_eq!(batch.points.len(), 1);
        clear_callback_state();
    }
}
