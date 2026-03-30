use std::{env, path::PathBuf, time::Duration};

use eyre::{Context, Result, ensure};

#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub sdk_config_path: PathBuf,
    pub publish_interval: Duration,
    pub lidar_frame_id: String,
    pub imu_frame_id: String,
}

impl DriverConfig {
    pub fn from_env() -> Result<Self> {
        let sdk_config_path = PathBuf::from(
            env::var("LIVOX_SDK2_CONFIG_PATH")
                .context("LIVOX_SDK2_CONFIG_PATH must point to a Livox-SDK2 JSON config")?,
        );
        ensure!(
            sdk_config_path.exists(),
            "LIVOX_SDK2_CONFIG_PATH does not exist: {}",
            sdk_config_path.display()
        );

        let publish_interval_ms = env::var("MID360_PUBLISH_INTERVAL_MS")
            .ok()
            .map(|value| value.parse::<u64>())
            .transpose()
            .context("MID360_PUBLISH_INTERVAL_MS must be an unsigned integer")?
            .unwrap_or(100);
        ensure!(
            publish_interval_ms > 0,
            "MID360_PUBLISH_INTERVAL_MS must be greater than zero"
        );

        Ok(Self {
            sdk_config_path,
            publish_interval: Duration::from_millis(publish_interval_ms),
            lidar_frame_id: env::var("MID360_LIDAR_FRAME_ID")
                .unwrap_or_else(|_| "livox_frame".to_owned()),
            imu_frame_id: env::var("MID360_IMU_FRAME_ID")
                .unwrap_or_else(|_| "livox_imu".to_owned()),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::env;

    use super::DriverConfig;

    #[test]
    fn defaults_are_applied() {
        let temp_dir = tempfile::tempdir().unwrap();
        let config_path = temp_dir.path().join("mid360.json");
        std::fs::write(&config_path, "{}").unwrap();

        unsafe {
            env::set_var("LIVOX_SDK2_CONFIG_PATH", &config_path);
            env::remove_var("MID360_PUBLISH_INTERVAL_MS");
            env::remove_var("MID360_LIDAR_FRAME_ID");
            env::remove_var("MID360_IMU_FRAME_ID");
        }

        let config = DriverConfig::from_env().unwrap();
        assert_eq!(config.publish_interval.as_millis(), 100);
        assert_eq!(config.lidar_frame_id, "livox_frame");
        assert_eq!(config.imu_frame_id, "livox_imu");
    }
}
