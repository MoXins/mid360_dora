use std::sync::Arc;

use arrow::array::{
    Array, ArrayRef, Float32Array, Float32Builder, GenericListBuilder, ListArray, StringArray,
    StringBuilder, StructArray, UInt8Array, UInt8Builder, UInt32Array, UInt32Builder, UInt64Array,
    UInt64Builder,
};
use arrow::datatypes::{DataType, Field, Fields};
use eyre::{Result, ensure};

#[derive(Debug, Clone, PartialEq)]
pub struct PointsFrame {
    pub lidar_ip: String,
    pub frame_id: String,
    pub stamp_ns: u64,
    pub point_count: u32,
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    pub z: Vec<f32>,
    pub intensity: Vec<f32>,
    pub tag: Vec<u8>,
    pub point_stamp_ns: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    pub lidar_ip: String,
    pub frame_id: String,
    pub stamp_ns: u64,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
}

impl PointsFrame {
    pub fn into_arrow(self) -> StructArray {
        let float_item = Field::new("item", DataType::Float32, false);
        let uint8_item = Field::new("item", DataType::UInt8, false);
        let uint64_item = Field::new("item", DataType::UInt64, false);

        let mut lidar_ip = StringBuilder::new();
        let mut frame_id = StringBuilder::new();
        let mut stamp_ns = UInt64Builder::new();
        let mut point_count = UInt32Builder::new();
        let mut x = GenericListBuilder::new(Float32Builder::new()).with_field(float_item.clone());
        let mut y = GenericListBuilder::new(Float32Builder::new()).with_field(float_item.clone());
        let mut z = GenericListBuilder::new(Float32Builder::new()).with_field(float_item.clone());
        let mut intensity =
            GenericListBuilder::new(Float32Builder::new()).with_field(float_item.clone());
        let mut tag = GenericListBuilder::new(UInt8Builder::new()).with_field(uint8_item);
        let mut point_stamp_ns =
            GenericListBuilder::new(UInt64Builder::new()).with_field(uint64_item);

        lidar_ip.append_value(self.lidar_ip);
        frame_id.append_value(self.frame_id);
        stamp_ns.append_value(self.stamp_ns);
        point_count.append_value(self.point_count);
        append_f32_list(&mut x, &self.x);
        append_f32_list(&mut y, &self.y);
        append_f32_list(&mut z, &self.z);
        append_f32_list(&mut intensity, &self.intensity);
        append_u8_list(&mut tag, &self.tag);
        append_u64_list(&mut point_stamp_ns, &self.point_stamp_ns);

        let fields = Fields::from(vec![
            Field::new("lidar_ip", DataType::Utf8, false),
            Field::new("frame_id", DataType::Utf8, false),
            Field::new("stamp_ns", DataType::UInt64, false),
            Field::new("point_count", DataType::UInt32, false),
            Field::new(
                "x",
                DataType::List(Arc::new(Field::new("item", DataType::Float32, false))),
                false,
            ),
            Field::new(
                "y",
                DataType::List(Arc::new(Field::new("item", DataType::Float32, false))),
                false,
            ),
            Field::new(
                "z",
                DataType::List(Arc::new(Field::new("item", DataType::Float32, false))),
                false,
            ),
            Field::new(
                "intensity",
                DataType::List(Arc::new(Field::new("item", DataType::Float32, false))),
                false,
            ),
            Field::new(
                "tag",
                DataType::List(Arc::new(Field::new("item", DataType::UInt8, false))),
                false,
            ),
            Field::new(
                "point_stamp_ns",
                DataType::List(Arc::new(Field::new("item", DataType::UInt64, false))),
                false,
            ),
        ]);

        StructArray::new(
            fields,
            vec![
                Arc::new(lidar_ip.finish()),
                Arc::new(frame_id.finish()),
                Arc::new(stamp_ns.finish()),
                Arc::new(point_count.finish()),
                Arc::new(x.finish()),
                Arc::new(y.finish()),
                Arc::new(z.finish()),
                Arc::new(intensity.finish()),
                Arc::new(tag.finish()),
                Arc::new(point_stamp_ns.finish()),
            ],
            None,
        )
    }

    pub fn try_from_array(array: &dyn Array) -> Result<Self> {
        let struct_array = array
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| eyre::eyre!("expected StructArray"))?;
        ensure!(struct_array.len() == 1, "expected exactly one row");

        let x = read_f32_list(struct_array, "x")?;
        let y = read_f32_list(struct_array, "y")?;
        let z = read_f32_list(struct_array, "z")?;
        let intensity = read_f32_list(struct_array, "intensity")?;
        let tag = read_u8_list(struct_array, "tag")?;
        let point_stamp_ns = read_u64_list(struct_array, "point_stamp_ns")?;

        let len = x.len();
        ensure!(y.len() == len, "y length mismatch");
        ensure!(z.len() == len, "z length mismatch");
        ensure!(intensity.len() == len, "intensity length mismatch");
        ensure!(tag.len() == len, "tag length mismatch");
        ensure!(
            point_stamp_ns.len() == len,
            "point_stamp_ns length mismatch"
        );

        Ok(Self {
            lidar_ip: read_string(struct_array, "lidar_ip")?,
            frame_id: read_string(struct_array, "frame_id")?,
            stamp_ns: read_u64(struct_array, "stamp_ns")?,
            point_count: read_u32(struct_array, "point_count")?,
            x,
            y,
            z,
            intensity,
            tag,
            point_stamp_ns,
        })
    }

    pub fn validate(&self) -> Result<()> {
        let len = self.x.len();
        ensure!(self.y.len() == len, "y length mismatch");
        ensure!(self.z.len() == len, "z length mismatch");
        ensure!(self.intensity.len() == len, "intensity length mismatch");
        ensure!(self.tag.len() == len, "tag length mismatch");
        ensure!(
            self.point_stamp_ns.len() == len,
            "point_stamp_ns length mismatch"
        );
        ensure!(self.point_count as usize == len, "point_count mismatch");
        Ok(())
    }
}

impl ImuSample {
    pub fn into_arrow(self) -> StructArray {
        let fields = Fields::from(vec![
            Field::new("lidar_ip", DataType::Utf8, false),
            Field::new("frame_id", DataType::Utf8, false),
            Field::new("stamp_ns", DataType::UInt64, false),
            Field::new("gyro_x", DataType::Float32, false),
            Field::new("gyro_y", DataType::Float32, false),
            Field::new("gyro_z", DataType::Float32, false),
            Field::new("acc_x", DataType::Float32, false),
            Field::new("acc_y", DataType::Float32, false),
            Field::new("acc_z", DataType::Float32, false),
        ]);
        StructArray::new(
            fields,
            vec![
                Arc::new(StringArray::from(vec![self.lidar_ip])),
                Arc::new(StringArray::from(vec![self.frame_id])),
                Arc::new(UInt64Array::from(vec![self.stamp_ns])),
                Arc::new(Float32Array::from(vec![self.gyro_x])),
                Arc::new(Float32Array::from(vec![self.gyro_y])),
                Arc::new(Float32Array::from(vec![self.gyro_z])),
                Arc::new(Float32Array::from(vec![self.acc_x])),
                Arc::new(Float32Array::from(vec![self.acc_y])),
                Arc::new(Float32Array::from(vec![self.acc_z])),
            ],
            None,
        )
    }

    pub fn try_from_array(array: &dyn Array) -> Result<Self> {
        let struct_array = array
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| eyre::eyre!("expected StructArray"))?;
        ensure!(struct_array.len() == 1, "expected exactly one row");

        Ok(Self {
            lidar_ip: read_string(struct_array, "lidar_ip")?,
            frame_id: read_string(struct_array, "frame_id")?,
            stamp_ns: read_u64(struct_array, "stamp_ns")?,
            gyro_x: read_f32(struct_array, "gyro_x")?,
            gyro_y: read_f32(struct_array, "gyro_y")?,
            gyro_z: read_f32(struct_array, "gyro_z")?,
            acc_x: read_f32(struct_array, "acc_x")?,
            acc_y: read_f32(struct_array, "acc_y")?,
            acc_z: read_f32(struct_array, "acc_z")?,
        })
    }
}

fn append_f32_list(builder: &mut GenericListBuilder<i32, Float32Builder>, values: &[f32]) {
    for value in values {
        builder.values().append_value(*value);
    }
    builder.append(true);
}

fn append_u8_list(builder: &mut GenericListBuilder<i32, UInt8Builder>, values: &[u8]) {
    for value in values {
        builder.values().append_value(*value);
    }
    builder.append(true);
}

fn append_u64_list(builder: &mut GenericListBuilder<i32, UInt64Builder>, values: &[u64]) {
    for value in values {
        builder.values().append_value(*value);
    }
    builder.append(true);
}

fn read_string(struct_array: &StructArray, name: &str) -> Result<String> {
    let array = struct_array
        .column_by_name(name)
        .ok_or_else(|| eyre::eyre!("missing field `{name}`"))?;
    let array = array
        .as_any()
        .downcast_ref::<StringArray>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not Utf8"))?;
    Ok(array.value(0).to_owned())
}

fn read_u64(struct_array: &StructArray, name: &str) -> Result<u64> {
    let array = struct_array
        .column_by_name(name)
        .ok_or_else(|| eyre::eyre!("missing field `{name}`"))?;
    let array = array
        .as_any()
        .downcast_ref::<UInt64Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not UInt64"))?;
    Ok(array.value(0))
}

fn read_u32(struct_array: &StructArray, name: &str) -> Result<u32> {
    let array = struct_array
        .column_by_name(name)
        .ok_or_else(|| eyre::eyre!("missing field `{name}`"))?;
    let array = array
        .as_any()
        .downcast_ref::<UInt32Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not UInt32"))?;
    Ok(array.value(0))
}

fn read_f32(struct_array: &StructArray, name: &str) -> Result<f32> {
    let array = struct_array
        .column_by_name(name)
        .ok_or_else(|| eyre::eyre!("missing field `{name}`"))?;
    let array = array
        .as_any()
        .downcast_ref::<Float32Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not Float32"))?;
    Ok(array.value(0))
}

fn read_f32_list(struct_array: &StructArray, name: &str) -> Result<Vec<f32>> {
    let values = read_list_values(struct_array, name)?;
    let values = values
        .as_any()
        .downcast_ref::<Float32Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not List<Float32>"))?;
    Ok(values.values().to_vec())
}

fn read_u8_list(struct_array: &StructArray, name: &str) -> Result<Vec<u8>> {
    let values = read_list_values(struct_array, name)?;
    let values = values
        .as_any()
        .downcast_ref::<UInt8Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not List<UInt8>"))?;
    Ok(values.values().to_vec())
}

fn read_u64_list(struct_array: &StructArray, name: &str) -> Result<Vec<u64>> {
    let values = read_list_values(struct_array, name)?;
    let values = values
        .as_any()
        .downcast_ref::<UInt64Array>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not List<UInt64>"))?;
    Ok(values.values().to_vec())
}

fn read_list_values(struct_array: &StructArray, name: &str) -> Result<ArrayRef> {
    let array = struct_array
        .column_by_name(name)
        .ok_or_else(|| eyre::eyre!("missing field `{name}`"))?;
    let list_array = array
        .as_any()
        .downcast_ref::<ListArray>()
        .ok_or_else(|| eyre::eyre!("field `{name}` is not List<_>"))?;
    ensure!(
        list_array.len() == 1,
        "expected exactly one row in `{name}`"
    );
    Ok(list_array.value(0))
}

#[cfg(test)]
mod tests {
    use super::{ImuSample, PointsFrame};

    #[test]
    fn points_round_trip_arrow() {
        let frame = PointsFrame {
            lidar_ip: "192.168.1.100".to_owned(),
            frame_id: "livox_frame".to_owned(),
            stamp_ns: 42,
            point_count: 2,
            x: vec![1.0, 2.0],
            y: vec![3.0, 4.0],
            z: vec![5.0, 6.0],
            intensity: vec![7.0, 8.0],
            tag: vec![0, 1],
            point_stamp_ns: vec![10, 11],
        };
        let arrow = frame.clone().into_arrow();
        let decoded = PointsFrame::try_from_array(&arrow).unwrap();
        assert_eq!(decoded, frame);
        decoded.validate().unwrap();
    }

    #[test]
    fn imu_round_trip_arrow() {
        let imu = ImuSample {
            lidar_ip: "192.168.1.100".to_owned(),
            frame_id: "livox_imu".to_owned(),
            stamp_ns: 42,
            gyro_x: 1.0,
            gyro_y: 2.0,
            gyro_z: 3.0,
            acc_x: 4.0,
            acc_y: 5.0,
            acc_z: 6.0,
        };
        let arrow = imu.clone().into_arrow();
        let decoded = ImuSample::try_from_array(&arrow).unwrap();
        assert_eq!(decoded, imu);
    }
}
