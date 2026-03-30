#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_char, c_void};

pub type livox_status = i32;

pub const SDK2_LINKED: bool = cfg!(livox_sdk2_linked);

pub const kLivoxLidarTypeMid360: u8 = 9;
pub const kLivoxLidarImuData: u8 = 0;
pub const kLivoxLidarCartesianCoordinateHighData: u8 = 0x01;
pub const kLivoxLidarCartesianCoordinateLowData: u8 = 0x02;
pub const kLivoxLidarSphericalCoordinateData: u8 = 0x03;

pub const kTimestampTypeNoSync: u8 = 0;
pub const kTimestampTypeGptpOrPtp: u8 = 1;
pub const kTimestampTypeGps: u8 = 2;

pub const kLivoxLidarStatusSuccess: livox_status = 0;
pub const kLivoxLidarStatusTimeout: livox_status = -4;
pub const kLivoxLidarNormal: u8 = 0x01;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarInfo {
    pub dev_type: u8,
    pub sn: [c_char; 16],
    pub lidar_ip: [c_char; 16],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct LivoxLidarEthernetPacket {
    pub version: u8,
    pub length: u16,
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: u8,
    pub time_type: u8,
    pub rsvd: [u8; 12],
    pub crc32: u32,
    pub timestamp: [u8; 8],
    pub data: [u8; 1],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarImuRawPoint {
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarCartesianHighRawPoint {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub reflectivity: u8,
    pub tag: u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarCartesianLowRawPoint {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub reflectivity: u8,
    pub tag: u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarSpherPoint {
    pub depth: u32,
    pub theta: u16,
    pub phi: u16,
    pub reflectivity: u8,
    pub tag: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LivoxLidarAsyncControlResponse {
    pub ret_code: u8,
    pub error_key: u16,
}

pub type LivoxLidarPointCloudCallBack =
    Option<unsafe extern "C" fn(u32, u8, *mut LivoxLidarEthernetPacket, *mut c_void)>;
pub type LivoxLidarImuDataCallback =
    Option<unsafe extern "C" fn(u32, u8, *mut LivoxLidarEthernetPacket, *mut c_void)>;
pub type LivoxLidarInfoChangeCallback =
    Option<unsafe extern "C" fn(u32, *const LivoxLidarInfo, *mut c_void)>;
pub type LivoxLidarAsyncControlCallback = Option<
    unsafe extern "C" fn(livox_status, u32, *mut LivoxLidarAsyncControlResponse, *mut c_void),
>;

#[cfg(livox_sdk2_linked)]
unsafe extern "C" {
    pub fn LivoxLidarSdkInit(
        path: *const c_char,
        host_ip: *const c_char,
        log_cfg_info: *const c_void,
    ) -> bool;
    pub fn LivoxLidarSdkUninit();
    pub fn SetLivoxLidarPointCloudCallBack(
        cb: LivoxLidarPointCloudCallBack,
        client_data: *mut c_void,
    );
    pub fn SetLivoxLidarImuDataCallback(cb: LivoxLidarImuDataCallback, client_data: *mut c_void);
    pub fn SetLivoxLidarInfoChangeCallback(
        cb: LivoxLidarInfoChangeCallback,
        client_data: *mut c_void,
    );
    pub fn SetLivoxLidarWorkMode(
        handle: u32,
        work_mode: u8,
        cb: LivoxLidarAsyncControlCallback,
        client_data: *mut c_void,
    ) -> livox_status;
    pub fn EnableLivoxLidarImuData(
        handle: u32,
        cb: LivoxLidarAsyncControlCallback,
        client_data: *mut c_void,
    ) -> livox_status;
    pub fn DisableLivoxSdkConsoleLogger();
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn LivoxLidarSdkInit(
    _path: *const c_char,
    _host_ip: *const c_char,
    _log_cfg_info: *const c_void,
) -> bool {
    false
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn LivoxLidarSdkUninit() {}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn SetLivoxLidarPointCloudCallBack(
    _cb: LivoxLidarPointCloudCallBack,
    _client_data: *mut c_void,
) {
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn SetLivoxLidarImuDataCallback(
    _cb: LivoxLidarImuDataCallback,
    _client_data: *mut c_void,
) {
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn SetLivoxLidarInfoChangeCallback(
    _cb: LivoxLidarInfoChangeCallback,
    _client_data: *mut c_void,
) {
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn SetLivoxLidarWorkMode(
    _handle: u32,
    _work_mode: u8,
    _cb: LivoxLidarAsyncControlCallback,
    _client_data: *mut c_void,
) -> livox_status {
    -1
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn EnableLivoxLidarImuData(
    _handle: u32,
    _cb: LivoxLidarAsyncControlCallback,
    _client_data: *mut c_void,
) -> livox_status {
    -1
}

#[cfg(not(livox_sdk2_linked))]
pub unsafe fn DisableLivoxSdkConsoleLogger() {}

pub const fn ethernet_header_size() -> usize {
    core::mem::offset_of!(LivoxLidarEthernetPacket, data)
}

pub fn handle_to_ipv4(handle: u32) -> std::net::Ipv4Addr {
    std::net::Ipv4Addr::from(handle.to_ne_bytes())
}

#[cfg(test)]
mod tests {
    use super::handle_to_ipv4;

    #[test]
    fn converts_sdk_handle_to_ip() {
        let handle = 0x0501_A8C0;
        assert_eq!(handle_to_ipv4(handle).to_string(), "192.168.1.5");
    }
}
