use crate::bridge::ffi::{
    ImuRust, LidarWorkingModeRust, MessageTypeRust, PointUnitreeRust, UnitreeLidarWrapper,
    make_unitree_lidar_reader,
};
use cxx::UniquePtr;
use futures_util::Stream;
use nalgebra::Vector3;
use std::pin::Pin;
use std::task::{Context, Poll};

pub struct LidarReader {
    reader: UniquePtr<UnitreeLidarWrapper>,
}

impl LidarReader {
    pub fn new_with_initialize(
        cloud_scan_num: u32,
        port: String,
        baudrate: u32,
        min_distance_meters: f64,
        max_distance_meters: f64,
        rotate_yaw_bias: f64,
        range_scale: f64,
        range_bias: f64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut reader = make_unitree_lidar_reader();

        let result = reader.pin_mut().initialize(
            cloud_scan_num as u16,
            &port,
            baudrate,
            rotate_yaw_bias as f32,
            range_scale as f32,
            range_bias as f32,
            max_distance_meters as f32,
            min_distance_meters as f32,
        );

        if !result {
            return Err("Failed to initialize LiDAR".into());
        }

        Ok(Self { reader })
    }

    pub fn new(
        port: String,
        min_distance_meters: f64,
        max_distance_meters: f64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        return Self::new_with_initialize(
            18,
            port,
            2000000,
            min_distance_meters,
            max_distance_meters,
            0.0,
            0.001,
            0.0,
        );
    }

    pub fn start_lidar(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.reader
            .pin_mut()
            .set_lidar_working_mode(LidarWorkingModeRust::Standby);
        std::thread::sleep(std::time::Duration::from_secs(1));
        self.reader
            .pin_mut()
            .set_lidar_working_mode(LidarWorkingModeRust::Normal);

        Ok(())
    }

    pub fn into_stream(self) -> LidarStream {
        LidarStream { reader: self }
    }
}

pub enum LidarResult {
    PointCloud(Vec<Vector3<f32>>),
    ImuReading(ImuRust),
}

pub struct LidarStream {
    reader: LidarReader,
}

impl Stream for LidarStream {
    type Item = LidarResult;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        match self.reader.reader.pin_mut().run_parse() {
            MessageTypeRust::PointCloud => {
                let points = self.reader.reader.pin_mut().get_cloud();
                if points.is_empty() {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                } else {
                    Poll::Ready(Some(LidarResult::PointCloud(
                        points.iter().map(|p| Vector3::new(p.x, p.y, p.z)).collect(),
                    )))
                }
            }
            MessageTypeRust::Imu => {
                let imu = self.reader.reader.pin_mut().get_imu();
                Poll::Ready(Some(LidarResult::ImuReading(imu)))
            }
            _ => {
                // If no data, wake up after a short delay
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }
}
