# MID-360 Dora Driver

This workspace provides a Dora-native MID-360 source node backed by `Livox-SDK2`.

## Crates

- `livox-sdk2-sys`: minimal FFI surface for the SDK2 C API.
- `mid360-dora-types`: Arrow schemas and codecs shared by the source node and downstream consumers.
- `mid360-dora-driver`: the source node that publishes `points` and `imu`.
- `examples/minimal-sink`: a tiny sink used to validate the emitted schema.

## Configuration

- `LIVOX_SDK2_CONFIG_PATH`: path to the official SDK2 JSON config.
- `MID360_PUBLISH_INTERVAL_MS`: frame publish interval in milliseconds. Default `100`.
- `MID360_LIDAR_FRAME_ID`: output frame id for `points`. Default `livox_frame`.
- `MID360_IMU_FRAME_ID`: output frame id for `imu`. Default `livox_imu`.
- `LIVOX_SDK2_INCLUDE_DIR` / `LIVOX_SDK2_LIB_DIR`: optional overrides for the installed SDK2 paths.

The committed `config/mid360_config.json` is only a template. Replace the IPs and ports with your actual MID-360 and host settings.

## Test

```bash
cd mid360-dora
CARGO_HOME=/tmp/mid360-cargo-home cargo test
```

When SDK2 is not installed, the FFI crate builds a stub backend so unit tests still run. A real sensor run still requires the shared library to be present.

## Run

Install `Livox-SDK2` first, then update `config/mid360_config.json` for your host IP.

```bash
cd mid360-dora
dora build dataflow.yml
dora run dataflow.yml
```
