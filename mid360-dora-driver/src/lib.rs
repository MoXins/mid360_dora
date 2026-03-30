mod config;
mod frame_accumulator;
mod packet_parser;
mod sdk_runtime;

use std::{
    sync::mpsc::{self, RecvTimeoutError},
    time::Instant,
};

use dora_node_api::{DoraNode, Event, dora_core::config::DataId};
use eyre::Result;
use futures::executor::block_on_stream;
use sdk_runtime::{DriverMessage, SdkRuntime};

use crate::{
    config::DriverConfig, frame_accumulator::FrameAccumulator, packet_parser::ParsedMessage,
};

pub fn run() -> Result<()> {
    let config = DriverConfig::from_env()?;
    let (mut node, dora_events) = DoraNode::init_from_env()?;
    let (tx, rx) = mpsc::sync_channel(1024);

    let _sdk = SdkRuntime::start(
        tx.clone(),
        config.sdk_config_path.to_string_lossy().as_ref(),
        config.imu_frame_id.clone(),
    )?;
    let dora_thread = spawn_dora_stop_forwarder(dora_events, tx.clone());

    let points_output = DataId::from("points".to_owned());
    let imu_output = DataId::from("imu".to_owned());
    let mut accumulator = FrameAccumulator::new();
    let mut next_flush = Instant::now() + config.publish_interval;

    loop {
        let wait = next_flush.saturating_duration_since(Instant::now());
        match rx.recv_timeout(wait) {
            Ok(DriverMessage::Parsed(ParsedMessage::PointsBatch(batch))) => {
                accumulator.push_batch(batch);
            }
            Ok(DriverMessage::Parsed(ParsedMessage::Imu(imu))) => {
                node.send_output(imu_output.clone(), Default::default(), imu.into_arrow())?;
            }
            Ok(DriverMessage::StopRequested) | Ok(DriverMessage::DoraClosed) => {
                flush_points(
                    &mut node,
                    &points_output,
                    &mut accumulator,
                    &config.lidar_frame_id,
                )?;
                break;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => {
                flush_points(
                    &mut node,
                    &points_output,
                    &mut accumulator,
                    &config.lidar_frame_id,
                )?;
                break;
            }
        }

        if Instant::now() >= next_flush {
            flush_points(
                &mut node,
                &points_output,
                &mut accumulator,
                &config.lidar_frame_id,
            )?;
            next_flush = Instant::now() + config.publish_interval;
        }
    }

    let _ = dora_thread.join();
    Ok(())
}

fn flush_points(
    node: &mut DoraNode,
    output_id: &DataId,
    accumulator: &mut FrameAccumulator,
    frame_id: &str,
) -> Result<()> {
    if let Some(frame) = accumulator.flush(frame_id)? {
        node.send_output(output_id.clone(), Default::default(), frame.into_arrow())?;
    }
    Ok(())
}

fn spawn_dora_stop_forwarder(
    dora_events: dora_node_api::EventStream,
    tx: mpsc::SyncSender<DriverMessage>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut events = block_on_stream(dora_events);
        while let Some(event) = events.next() {
            match event {
                Event::Stop(_) => {
                    let _ = tx.send(DriverMessage::StopRequested);
                    return;
                }
                Event::Error(err) => {
                    eprintln!("Dora runtime error: {err}");
                }
                other => {
                    eprintln!("Ignoring unexpected Dora event: {other:?}");
                }
            }
        }
        let _ = tx.send(DriverMessage::DoraClosed);
    })
}
