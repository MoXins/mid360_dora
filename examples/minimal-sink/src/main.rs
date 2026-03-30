use dora_node_api::{DoraNode, Event};
use eyre::Result;
use mid360_dora_types::{ImuSample, PointsFrame};

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "points" => {
                    let frame = PointsFrame::try_from_array(data.as_ref())?;
                    frame.validate()?;
                    println!(
                        "received points frame from {} with {} points",
                        frame.lidar_ip, frame.point_count
                    );
                }
                "imu" => {
                    let imu = ImuSample::try_from_array(data.as_ref())?;
                    println!("received imu from {} at {}", imu.lidar_ip, imu.stamp_ns);
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop(_) => break,
            other => eprintln!("Ignoring unexpected event: {other:?}"),
        }
    }

    Ok(())
}
