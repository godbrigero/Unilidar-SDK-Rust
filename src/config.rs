/// How the lidar is connected to this machine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Connection {
    /// USB / serial connection (the default for the L1).
    Serial { port: String, baudrate: u32 },
    /// Connection through the Unitree UDP board.
    Udp {
        lidar_ip: String,
        lidar_port: u16,
        local_ip: String,
        local_port: u16,
    },
}

/// Settings used by [`Lidar::open`](crate::Lidar::open).
///
/// Start from [`Config::serial`] or [`Config::udp`] and override what you need.
/// All other values default to the ones used by the official SDK.
///
/// ```no_run
/// use unitree_lidar_l1_rust::Config;
///
/// let config = Config::serial("/dev/ttyUSB0").range(0.2, 30.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub connection: Connection,
    /// Number of scans accumulated into one [`PointCloud`](crate::PointCloud).
    pub cloud_scan_num: u16,
    /// Points closer than this (meters) are dropped.
    pub range_min: f32,
    /// Points further than this (meters) are dropped.
    pub range_max: f32,
    /// Scale applied to raw range values.
    pub range_scale: f32,
    /// Bias (meters) added to range values.
    pub range_bias: f32,
    /// Yaw offset (radians) applied to every point.
    pub rotate_yaw_bias: f32,
    /// Whether [`Lidar::open`](crate::Lidar::open) starts the motor right away.
    pub start_on_open: bool,
}

impl Config {
    pub const DEFAULT_BAUDRATE: u32 = 2_000_000;

    /// Serial connection on `port` (e.g. `/dev/ttyUSB0`) at the default baudrate.
    pub fn serial(port: impl Into<String>) -> Self {
        Self::with_connection(Connection::Serial {
            port: port.into(),
            baudrate: Self::DEFAULT_BAUDRATE,
        })
    }

    /// UDP connection using the SDK's default addresses
    /// (lidar `10.10.10.10:5001`, local `10.10.10.100:5000`).
    pub fn udp() -> Self {
        Self::with_connection(Connection::Udp {
            lidar_ip: "10.10.10.10".into(),
            lidar_port: 5001,
            local_ip: "10.10.10.100".into(),
            local_port: 5000,
        })
    }

    pub fn with_connection(connection: Connection) -> Self {
        Self {
            connection,
            cloud_scan_num: 18,
            range_min: 0.0,
            range_max: 50.0,
            range_scale: 0.001,
            range_bias: 0.0,
            rotate_yaw_bias: 0.0,
            start_on_open: true,
        }
    }

    /// Serial baudrate. Ignored for UDP connections.
    pub fn baudrate(mut self, baudrate: u32) -> Self {
        if let Connection::Serial { baudrate: b, .. } = &mut self.connection {
            *b = baudrate;
        }
        self
    }

    /// Lidar address for UDP connections. Ignored for serial connections.
    pub fn lidar_addr(mut self, ip: impl Into<String>, port: u16) -> Self {
        if let Connection::Udp {
            lidar_ip, lidar_port, ..
        } = &mut self.connection
        {
            *lidar_ip = ip.into();
            *lidar_port = port;
        }
        self
    }

    /// Local address for UDP connections. Ignored for serial connections.
    pub fn local_addr(mut self, ip: impl Into<String>, port: u16) -> Self {
        if let Connection::Udp {
            local_ip, local_port, ..
        } = &mut self.connection
        {
            *local_ip = ip.into();
            *local_port = port;
        }
        self
    }

    /// Only keep points between `min` and `max` meters.
    pub fn range(mut self, min: f32, max: f32) -> Self {
        self.range_min = min;
        self.range_max = max;
        self
    }

    pub fn cloud_scan_num(mut self, cloud_scan_num: u16) -> Self {
        self.cloud_scan_num = cloud_scan_num;
        self
    }

    pub fn range_scale(mut self, range_scale: f32) -> Self {
        self.range_scale = range_scale;
        self
    }

    pub fn range_bias(mut self, range_bias: f32) -> Self {
        self.range_bias = range_bias;
        self
    }

    pub fn rotate_yaw_bias(mut self, rotate_yaw_bias: f32) -> Self {
        self.rotate_yaw_bias = rotate_yaw_bias;
        self
    }

    pub fn start_on_open(mut self, start_on_open: bool) -> Self {
        self.start_on_open = start_on_open;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), String> {
        if !(self.range_min >= 0.0 && self.range_min < self.range_max) {
            return Err(format!(
                "range must satisfy 0 <= min < max (got min={}, max={})",
                self.range_min, self.range_max
            ));
        }
        if self.cloud_scan_num == 0 {
            return Err("cloud_scan_num must be greater than 0".into());
        }
        Ok(())
    }
}
