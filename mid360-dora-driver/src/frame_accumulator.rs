use eyre::Result;
use mid360_dora_types::PointsFrame;

use crate::packet_parser::PointsBatch;

#[derive(Debug, Default)]
pub struct FrameAccumulator {
    lidar_ip: Option<String>,
    points: Vec<crate::packet_parser::PointSample>,
}

impl FrameAccumulator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_batch(&mut self, batch: PointsBatch) {
        if self.lidar_ip.is_none() {
            self.lidar_ip = Some(batch.lidar_ip.clone());
        }
        self.points.extend(batch.points);
    }

    pub fn flush(&mut self, frame_id: &str) -> Result<Option<PointsFrame>> {
        if self.points.is_empty() {
            return Ok(None);
        }

        let lidar_ip = self
            .lidar_ip
            .clone()
            .unwrap_or_else(|| "0.0.0.0".to_owned());
        let stamp_ns = self
            .points
            .iter()
            .map(|point| point.stamp_ns)
            .min()
            .unwrap_or_default();

        let mut x = Vec::with_capacity(self.points.len());
        let mut y = Vec::with_capacity(self.points.len());
        let mut z = Vec::with_capacity(self.points.len());
        let mut intensity = Vec::with_capacity(self.points.len());
        let mut tag = Vec::with_capacity(self.points.len());
        let mut point_stamp_ns = Vec::with_capacity(self.points.len());

        for point in self.points.drain(..) {
            x.push(point.x);
            y.push(point.y);
            z.push(point.z);
            intensity.push(point.intensity);
            tag.push(point.tag);
            point_stamp_ns.push(point.stamp_ns);
        }

        let frame = PointsFrame {
            lidar_ip,
            frame_id: frame_id.to_owned(),
            stamp_ns,
            point_count: x.len() as u32,
            x,
            y,
            z,
            intensity,
            tag,
            point_stamp_ns,
        };
        frame.validate()?;
        Ok(Some(frame))
    }
}

#[cfg(test)]
mod tests {
    use crate::frame_accumulator::FrameAccumulator;
    use crate::packet_parser::{PointSample, PointsBatch};

    #[test]
    fn flushes_non_empty_frame() {
        let mut accumulator = FrameAccumulator::new();
        accumulator.push_batch(PointsBatch {
            lidar_ip: "192.168.1.100".to_owned(),
            points: vec![
                PointSample {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    intensity: 4.0,
                    tag: 0,
                    stamp_ns: 200,
                },
                PointSample {
                    x: 5.0,
                    y: 6.0,
                    z: 7.0,
                    intensity: 8.0,
                    tag: 1,
                    stamp_ns: 100,
                },
            ],
        });

        let frame = accumulator.flush("livox_frame").unwrap().unwrap();
        assert_eq!(frame.point_count, 2);
        assert_eq!(frame.stamp_ns, 100);
        assert_eq!(frame.lidar_ip, "192.168.1.100");
        assert!(accumulator.flush("livox_frame").unwrap().is_none());
    }
}
