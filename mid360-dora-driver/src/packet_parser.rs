use std::{
    collections::HashMap,
    f64::consts::PI,
    net::Ipv4Addr,
    ptr, slice,
    time::{SystemTime, UNIX_EPOCH},
};

use eyre::{Result, bail};
use livox_sdk2_sys::{
    LivoxLidarCartesianHighRawPoint, LivoxLidarCartesianLowRawPoint, LivoxLidarEthernetPacket,
    LivoxLidarImuRawPoint, LivoxLidarSpherPoint, ethernet_header_size,
    kLivoxLidarCartesianCoordinateHighData, kLivoxLidarCartesianCoordinateLowData,
    kLivoxLidarImuData, kLivoxLidarSphericalCoordinateData, kTimestampTypeNoSync,
};
use mid360_dora_types::ImuSample;

#[derive(Debug, Clone, PartialEq)]
pub struct PointSample {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub tag: u8,
    pub stamp_ns: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PointsBatch {
    pub lidar_ip: String,
    pub points: Vec<PointSample>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ParsedMessage {
    PointsBatch(PointsBatch),
    Imu(ImuSample),
}

#[derive(Debug, Default)]
pub struct TimeSyncTracker {
    offsets_ns: HashMap<Ipv4Addr, i128>,
}

impl TimeSyncTracker {
    pub fn adjust_timestamp_ns(
        &mut self,
        lidar_ip: Ipv4Addr,
        time_type: u8,
        stamp_ns: u64,
        now_ns: u64,
    ) -> u64 {
        if time_type != kTimestampTypeNoSync {
            return stamp_ns;
        }

        let offset = self
            .offsets_ns
            .entry(lidar_ip)
            .or_insert(now_ns as i128 - stamp_ns as i128);
        (stamp_ns as i128 + *offset).max(0) as u64
    }

    pub fn now_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }
}

pub unsafe fn parse_packet_from_ffi(
    tracker: &mut TimeSyncTracker,
    lidar_ip: Ipv4Addr,
    frame_id: &str,
    packet: *const LivoxLidarEthernetPacket,
) -> Result<Option<ParsedMessage>> {
    if packet.is_null() {
        return Ok(None);
    }

    let length = read_u16(unsafe { ptr::addr_of!((*packet).length) });
    let payload_len = (length as usize).saturating_sub(ethernet_header_size());
    let payload =
        unsafe { slice::from_raw_parts(ptr::addr_of!((*packet).data).cast::<u8>(), payload_len) };

    let dot_num = read_u16(unsafe { ptr::addr_of!((*packet).dot_num) }) as usize;
    let time_interval = read_u16(unsafe { ptr::addr_of!((*packet).time_interval) });
    let data_type = read_u8(unsafe { ptr::addr_of!((*packet).data_type) });
    let time_type = read_u8(unsafe { ptr::addr_of!((*packet).time_type) });
    let stamp_ns = read_u64_from_bytes(unsafe { ptr::addr_of!((*packet).timestamp).cast::<u8>() });
    let adjusted_stamp_ns =
        tracker.adjust_timestamp_ns(lidar_ip, time_type, stamp_ns, TimeSyncTracker::now_ns());

    match data_type {
        value if value == kLivoxLidarCartesianCoordinateHighData => {
            parse_cartesian_high(lidar_ip, dot_num, time_interval, adjusted_stamp_ns, payload)
                .map(|batch| batch.map(ParsedMessage::PointsBatch))
        }
        value if value == kLivoxLidarCartesianCoordinateLowData => {
            parse_cartesian_low(lidar_ip, dot_num, time_interval, adjusted_stamp_ns, payload)
                .map(|batch| batch.map(ParsedMessage::PointsBatch))
        }
        value if value == kLivoxLidarSphericalCoordinateData => {
            parse_spherical(lidar_ip, dot_num, time_interval, adjusted_stamp_ns, payload)
                .map(|batch| batch.map(ParsedMessage::PointsBatch))
        }
        value if value == kLivoxLidarImuData => {
            parse_imu(lidar_ip, frame_id, adjusted_stamp_ns, payload)
                .map(|imu| imu.map(ParsedMessage::Imu))
        }
        other => bail!("unsupported Livox packet data type: {other}"),
    }
}

fn parse_cartesian_high(
    lidar_ip: Ipv4Addr,
    dot_num: usize,
    time_interval: u16,
    stamp_ns: u64,
    payload: &[u8],
) -> Result<Option<PointsBatch>> {
    let raw_points = cast_payload::<LivoxLidarCartesianHighRawPoint>(payload, dot_num)?;
    let point_interval_ns = point_interval_ns(time_interval, dot_num);

    let mut points = Vec::with_capacity(dot_num);
    for (idx, raw_point) in raw_points.iter().enumerate() {
        if !is_valid_tag(raw_point.tag) {
            continue;
        }
        points.push(PointSample {
            x: raw_point.x as f32 / 1000.0,
            y: raw_point.y as f32 / 1000.0,
            z: raw_point.z as f32 / 1000.0,
            intensity: raw_point.reflectivity as f32,
            tag: raw_point.tag,
            stamp_ns: stamp_ns + point_interval_ns * idx as u64,
        });
    }
    Ok((!points.is_empty()).then_some(PointsBatch {
        lidar_ip: lidar_ip.to_string(),
        points,
    }))
}

fn parse_cartesian_low(
    lidar_ip: Ipv4Addr,
    dot_num: usize,
    time_interval: u16,
    stamp_ns: u64,
    payload: &[u8],
) -> Result<Option<PointsBatch>> {
    let raw_points = cast_payload::<LivoxLidarCartesianLowRawPoint>(payload, dot_num)?;
    let point_interval_ns = point_interval_ns(time_interval, dot_num);

    let mut points = Vec::with_capacity(dot_num);
    for (idx, raw_point) in raw_points.iter().enumerate() {
        if !is_valid_tag(raw_point.tag) {
            continue;
        }
        points.push(PointSample {
            x: raw_point.x as f32 / 100.0,
            y: raw_point.y as f32 / 100.0,
            z: raw_point.z as f32 / 100.0,
            intensity: raw_point.reflectivity as f32,
            tag: raw_point.tag,
            stamp_ns: stamp_ns + point_interval_ns * idx as u64,
        });
    }
    Ok((!points.is_empty()).then_some(PointsBatch {
        lidar_ip: lidar_ip.to_string(),
        points,
    }))
}

fn parse_spherical(
    lidar_ip: Ipv4Addr,
    dot_num: usize,
    time_interval: u16,
    stamp_ns: u64,
    payload: &[u8],
) -> Result<Option<PointsBatch>> {
    let raw_points = cast_payload::<LivoxLidarSpherPoint>(payload, dot_num)?;
    let point_interval_ns = point_interval_ns(time_interval, dot_num);

    let mut points = Vec::with_capacity(dot_num);
    for (idx, raw_point) in raw_points.iter().enumerate() {
        if !is_valid_tag(raw_point.tag) {
            continue;
        }
        let radius = raw_point.depth as f64 / 1000.0;
        let theta = raw_point.theta as f64 / 100.0 / 180.0 * PI;
        let phi = raw_point.phi as f64 / 100.0 / 180.0 * PI;
        points.push(PointSample {
            x: (radius * theta.sin() * phi.cos()) as f32,
            y: (radius * theta.sin() * phi.sin()) as f32,
            z: (radius * theta.cos()) as f32,
            intensity: raw_point.reflectivity as f32,
            tag: raw_point.tag,
            stamp_ns: stamp_ns + point_interval_ns * idx as u64,
        });
    }
    Ok((!points.is_empty()).then_some(PointsBatch {
        lidar_ip: lidar_ip.to_string(),
        points,
    }))
}

fn parse_imu(
    lidar_ip: Ipv4Addr,
    frame_id: &str,
    stamp_ns: u64,
    payload: &[u8],
) -> Result<Option<ImuSample>> {
    let raw_imu = cast_payload::<LivoxLidarImuRawPoint>(payload, 1)?
        .first()
        .copied()
        .ok_or_else(|| eyre::eyre!("empty imu payload"))?;
    Ok(Some(ImuSample {
        lidar_ip: lidar_ip.to_string(),
        frame_id: frame_id.to_owned(),
        stamp_ns,
        gyro_x: raw_imu.gyro_x,
        gyro_y: raw_imu.gyro_y,
        gyro_z: raw_imu.gyro_z,
        acc_x: raw_imu.acc_x,
        acc_y: raw_imu.acc_y,
        acc_z: raw_imu.acc_z,
    }))
}

fn point_interval_ns(time_interval: u16, dot_num: usize) -> u64 {
    if dot_num == 0 {
        0
    } else {
        (time_interval as u64 * 100) / dot_num as u64
    }
}

fn is_valid_tag(tag: u8) -> bool {
    (tag & 0b0011_1111) == 0
}

fn cast_payload<T: Copy>(payload: &[u8], count: usize) -> Result<&[T]> {
    let expected = std::mem::size_of::<T>() * count;
    if payload.len() < expected {
        bail!(
            "payload too short: expected at least {expected} bytes, got {}",
            payload.len()
        );
    }

    let ptr = payload.as_ptr().cast::<T>();
    Ok(unsafe { slice::from_raw_parts(ptr, count) })
}

fn read_u8(ptr: *const u8) -> u8 {
    unsafe { ptr.read_unaligned() }
}

fn read_u16(ptr: *const u16) -> u16 {
    u16::from_le(unsafe { ptr.read_unaligned() })
}

fn read_u64_from_bytes(ptr: *const u8) -> u64 {
    let mut timestamp = [0u8; 8];
    unsafe { ptr.copy_to_nonoverlapping(timestamp.as_mut_ptr(), 8) };
    u64::from_le_bytes(timestamp)
}

#[cfg(test)]
mod tests {
    use std::net::Ipv4Addr;

    use livox_sdk2_sys::{
        LivoxLidarCartesianHighRawPoint, LivoxLidarCartesianLowRawPoint, LivoxLidarEthernetPacket,
        LivoxLidarImuRawPoint, LivoxLidarSpherPoint, ethernet_header_size,
        kLivoxLidarCartesianCoordinateHighData, kLivoxLidarCartesianCoordinateLowData,
        kLivoxLidarImuData, kLivoxLidarSphericalCoordinateData, kTimestampTypeNoSync,
    };

    use crate::packet_parser::{ParsedMessage, TimeSyncTracker, parse_packet_from_ffi};

    fn build_packet<T: Copy>(data_type: u8, time_type: u8, stamp_ns: u64, points: &[T]) -> Vec<u8> {
        let header_len = ethernet_header_size();
        let payload_len = std::mem::size_of_val(points);
        let total_len = header_len + payload_len;
        let mut buffer = vec![0u8; total_len];

        buffer[0] = 0;
        buffer[1..3].copy_from_slice(&(total_len as u16).to_le_bytes());
        buffer[3..5].copy_from_slice(&(960u16).to_le_bytes());
        buffer[5..7].copy_from_slice(&(points.len() as u16).to_le_bytes());
        buffer[7..9].copy_from_slice(&0u16.to_le_bytes());
        buffer[9] = 0;
        buffer[10] = data_type;
        buffer[11] = time_type;
        buffer[28..32].copy_from_slice(&0u32.to_le_bytes());
        buffer[32..40].copy_from_slice(&stamp_ns.to_le_bytes());
        unsafe {
            let dst = buffer[header_len..].as_mut_ptr().cast::<T>();
            dst.copy_from_nonoverlapping(points.as_ptr(), points.len());
        }
        buffer
    }

    #[test]
    fn parses_cartesian_high_packet() {
        let packet = build_packet(
            kLivoxLidarCartesianCoordinateHighData,
            1,
            1_000,
            &[LivoxLidarCartesianHighRawPoint {
                x: 1000,
                y: 2000,
                z: 3000,
                reflectivity: 42,
                tag: 0,
            }],
        );
        let mut tracker = TimeSyncTracker::default();
        let message = unsafe {
            parse_packet_from_ffi(
                &mut tracker,
                Ipv4Addr::new(192, 168, 1, 100),
                "imu",
                packet.as_ptr().cast::<LivoxLidarEthernetPacket>(),
            )
        }
        .unwrap()
        .unwrap();

        let ParsedMessage::PointsBatch(batch) = message else {
            panic!("expected point batch");
        };
        assert_eq!(batch.points[0].x, 1.0);
        assert_eq!(batch.points[0].y, 2.0);
        assert_eq!(batch.points[0].z, 3.0);
    }

    #[test]
    fn parses_cartesian_low_packet() {
        let packet = build_packet(
            kLivoxLidarCartesianCoordinateLowData,
            1,
            1_000,
            &[LivoxLidarCartesianLowRawPoint {
                x: 100,
                y: 200,
                z: 300,
                reflectivity: 42,
                tag: 0,
            }],
        );
        let mut tracker = TimeSyncTracker::default();
        let message = unsafe {
            parse_packet_from_ffi(
                &mut tracker,
                Ipv4Addr::new(192, 168, 1, 100),
                "imu",
                packet.as_ptr().cast::<LivoxLidarEthernetPacket>(),
            )
        }
        .unwrap()
        .unwrap();

        let ParsedMessage::PointsBatch(batch) = message else {
            panic!("expected point batch");
        };
        assert_eq!(batch.points[0].x, 1.0);
        assert_eq!(batch.points[0].y, 2.0);
        assert_eq!(batch.points[0].z, 3.0);
    }

    #[test]
    fn parses_spherical_packet() {
        let packet = build_packet(
            kLivoxLidarSphericalCoordinateData,
            1,
            1_000,
            &[LivoxLidarSpherPoint {
                depth: 1000,
                theta: 9000,
                phi: 0,
                reflectivity: 42,
                tag: 0,
            }],
        );
        let mut tracker = TimeSyncTracker::default();
        let message = unsafe {
            parse_packet_from_ffi(
                &mut tracker,
                Ipv4Addr::new(192, 168, 1, 100),
                "imu",
                packet.as_ptr().cast::<LivoxLidarEthernetPacket>(),
            )
        }
        .unwrap()
        .unwrap();

        let ParsedMessage::PointsBatch(batch) = message else {
            panic!("expected point batch");
        };
        assert!((batch.points[0].x - 1.0).abs() < 1e-6);
        assert!(batch.points[0].y.abs() < 1e-6);
        assert!(batch.points[0].z.abs() < 1e-6);
    }

    #[test]
    fn parses_imu_packet() {
        let packet = build_packet(
            kLivoxLidarImuData,
            1,
            1_000,
            &[LivoxLidarImuRawPoint {
                gyro_x: 1.0,
                gyro_y: 2.0,
                gyro_z: 3.0,
                acc_x: 4.0,
                acc_y: 5.0,
                acc_z: 6.0,
            }],
        );
        let mut tracker = TimeSyncTracker::default();
        let message = unsafe {
            parse_packet_from_ffi(
                &mut tracker,
                Ipv4Addr::new(192, 168, 1, 100),
                "livox_imu",
                packet.as_ptr().cast::<LivoxLidarEthernetPacket>(),
            )
        }
        .unwrap()
        .unwrap();

        let ParsedMessage::Imu(imu) = message else {
            panic!("expected imu");
        };
        assert_eq!(imu.gyro_x, 1.0);
        assert_eq!(imu.acc_z, 6.0);
    }

    #[test]
    fn no_sync_uses_stable_delta() {
        let mut tracker = TimeSyncTracker::default();
        let lidar_ip = Ipv4Addr::new(192, 168, 1, 100);
        let first = tracker.adjust_timestamp_ns(lidar_ip, kTimestampTypeNoSync, 100, 1_000);
        let second = tracker.adjust_timestamp_ns(lidar_ip, kTimestampTypeNoSync, 200, 5_000);
        assert_eq!(first, 1_000);
        assert_eq!(second, 1_100);
    }

    #[test]
    fn invalid_tags_are_filtered() {
        let packet = build_packet(
            kLivoxLidarCartesianCoordinateHighData,
            1,
            1_000,
            &[
                LivoxLidarCartesianHighRawPoint {
                    x: 1000,
                    y: 2000,
                    z: 3000,
                    reflectivity: 42,
                    tag: 0,
                },
                LivoxLidarCartesianHighRawPoint {
                    x: 4000,
                    y: 5000,
                    z: 6000,
                    reflectivity: 7,
                    tag: 0b0000_0001,
                },
            ],
        );
        let mut tracker = TimeSyncTracker::default();
        let message = unsafe {
            parse_packet_from_ffi(
                &mut tracker,
                Ipv4Addr::new(192, 168, 1, 100),
                "imu",
                packet.as_ptr().cast::<LivoxLidarEthernetPacket>(),
            )
        }
        .unwrap()
        .unwrap();

        let ParsedMessage::PointsBatch(batch) = message else {
            panic!("expected point batch");
        };
        assert_eq!(batch.points.len(), 1);
    }
}
