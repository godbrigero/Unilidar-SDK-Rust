use crate::bridge::ffi;

/// Something the lidar produced.
#[derive(Debug, Clone)]
pub enum Event {
    Cloud(PointCloud),
    Imu(Imu),
}

/// One lidar return.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    /// Time (seconds) of this point relative to [`PointCloud::stamp`].
    pub time: f32,
    pub ring: u32,
}

impl Point {
    pub fn xyz(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    #[cfg(feature = "nalgebra")]
    pub fn to_vector3(&self) -> nalgebra::Vector3<f32> {
        nalgebra::Vector3::new(self.x, self.y, self.z)
    }
}

/// A full point cloud made of [`Config::cloud_scan_num`](crate::Config::cloud_scan_num) scans.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointCloud {
    /// Timestamp in seconds.
    pub stamp: f64,
    /// Sequence id.
    pub id: u32,
    pub ring_num: u32,
    pub points: Vec<Point>,
}

impl PointCloud {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    #[cfg(feature = "nalgebra")]
    pub fn to_vectors(&self) -> Vec<nalgebra::Vector3<f32>> {
        self.points.iter().map(Point::to_vector3).collect()
    }
}

/// One reading from the built-in IMU.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Imu {
    /// Timestamp in seconds.
    pub stamp: f64,
    /// Sequence id.
    pub id: u32,
    /// Orientation as `[x, y, z, w]`.
    pub quaternion: [f32; 4],
    /// rad/s
    pub angular_velocity: [f32; 3],
    /// m/s²
    pub linear_acceleration: [f32; 3],
}

impl Imu {
    #[cfg(feature = "nalgebra")]
    pub fn orientation(&self) -> nalgebra::UnitQuaternion<f32> {
        let [x, y, z, w] = self.quaternion;
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z))
    }

    #[cfg(feature = "nalgebra")]
    pub fn angular_velocity_vector(&self) -> nalgebra::Vector3<f32> {
        self.angular_velocity.into()
    }

    #[cfg(feature = "nalgebra")]
    pub fn linear_acceleration_vector(&self) -> nalgebra::Vector3<f32> {
        self.linear_acceleration.into()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkingMode {
    Normal,
    Standby,
}

/// Built-in LED ring animations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedMode {
    ForwardSlow,
    ForwardFast,
    ReverseSlow,
    ReverseFast,
    TripleFlip,
    TripleBreathing,
    SixStageBreathing,
}

impl From<ffi::CloudRust> for PointCloud {
    fn from(cloud: ffi::CloudRust) -> Self {
        Self {
            stamp: cloud.stamp,
            id: cloud.id,
            ring_num: cloud.ring_num,
            points: cloud
                .points
                .iter()
                .map(|p| Point {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                    intensity: p.intensity,
                    time: p.time,
                    ring: p.ring,
                })
                .collect(),
        }
    }
}

impl From<ffi::ImuRust> for Imu {
    fn from(imu: ffi::ImuRust) -> Self {
        Self {
            stamp: imu.stamp,
            id: imu.id,
            quaternion: imu.quaternion,
            angular_velocity: imu.angular_velocity,
            linear_acceleration: imu.linear_acceleration,
        }
    }
}

impl From<WorkingMode> for ffi::LidarWorkingModeRust {
    fn from(mode: WorkingMode) -> Self {
        match mode {
            WorkingMode::Normal => Self::Normal,
            WorkingMode::Standby => Self::Standby,
        }
    }
}

impl From<LedMode> for ffi::LedDisplayModeRust {
    fn from(mode: LedMode) -> Self {
        match mode {
            LedMode::ForwardSlow => Self::ForwardSlow,
            LedMode::ForwardFast => Self::ForwardFast,
            LedMode::ReverseSlow => Self::ReverseSlow,
            LedMode::ReverseFast => Self::ReverseFast,
            LedMode::TripleFlip => Self::TripleFlip,
            LedMode::TripleBreathing => Self::TripleBreathing,
            LedMode::SixStageBreathing => Self::SixStageBreathing,
        }
    }
}
