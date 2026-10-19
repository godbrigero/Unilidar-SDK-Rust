#[cxx::bridge]
#[allow(clippy::too_many_arguments)]
pub(crate) mod ffi {
    #[derive(Debug, Clone)]
    pub struct PointUnitreeRust {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub intensity: f32,
        pub time: f32,
        pub ring: u32,
    }

    #[derive(Debug, Clone)]
    pub struct CloudRust {
        pub stamp: f64,
        pub id: u32,
        pub ring_num: u32,
        pub points: Vec<PointUnitreeRust>,
    }

    #[derive(Debug, Clone)]
    pub struct ImuRust {
        pub stamp: f64,
        pub id: u32,
        pub quaternion: [f32; 4],
        pub angular_velocity: [f32; 3],
        pub linear_acceleration: [f32; 3],
    }

    #[derive(Debug, Clone, Copy)]
    pub enum LidarWorkingModeRust {
        Normal = 1,
        Standby = 2,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum LedDisplayModeRust {
        ForwardSlow = 2,
        ForwardFast = 3,
        ReverseSlow = 4,
        ReverseFast = 5,
        TripleFlip = 6,
        TripleBreathing = 7,
        SixStageBreathing = 8,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum MessageTypeRust {
        None = 0,
        Imu = 1,
        PointCloud = 2,
        Range = 3,
        Auxiliary = 4,
        Version = 5,
        TimeSync = 6,
    }

    unsafe extern "C++" {
        include!("unitree_lidar_wrapper.h");

        type UnitreeLidarWrapper;

        fn make_unitree_lidar_reader() -> UniquePtr<UnitreeLidarWrapper>;

        fn initialize(
            self: Pin<&mut UnitreeLidarWrapper>,
            cloud_scan_num: u16,
            port: &str,
            baudrate: u32,
            rotate_yaw_bias: f32,
            range_scale: f32,
            range_bias: f32,
            range_max: f32,
            range_min: f32,
        ) -> bool;

        fn initialize_udp(
            self: Pin<&mut UnitreeLidarWrapper>,
            cloud_scan_num: u16,
            lidar_port: u16,
            lidar_ip: &str,
            local_port: u16,
            local_ip: &str,
            rotate_yaw_bias: f32,
            range_scale: f32,
            range_bias: f32,
            range_max: f32,
            range_min: f32,
        ) -> bool;

        fn close_udp(self: Pin<&mut UnitreeLidarWrapper>) -> bool;

        fn run_parse(self: Pin<&mut UnitreeLidarWrapper>) -> MessageTypeRust;
        fn get_cloud(self: Pin<&mut UnitreeLidarWrapper>) -> CloudRust;
        fn get_imu(self: Pin<&mut UnitreeLidarWrapper>) -> ImuRust;
        fn reset(self: Pin<&mut UnitreeLidarWrapper>);
        fn set_lidar_working_mode(self: Pin<&mut UnitreeLidarWrapper>, mode: LidarWorkingModeRust);
        fn set_led_display_mode(self: Pin<&mut UnitreeLidarWrapper>, mode: LedDisplayModeRust);
        fn set_led_table(self: Pin<&mut UnitreeLidarWrapper>, table: &[u8]);
        fn get_version_of_firmware(self: Pin<&mut UnitreeLidarWrapper>) -> String;
        fn get_version_of_sdk(self: Pin<&mut UnitreeLidarWrapper>) -> String;
        fn get_time_delay(self: Pin<&mut UnitreeLidarWrapper>) -> u32;
        fn get_dirty_percentage(self: Pin<&mut UnitreeLidarWrapper>) -> f32;
    }
}
