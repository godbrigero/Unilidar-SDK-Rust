use crate::bridge::ffi::{self, MessageTypeRust, UnitreeLidarWrapper};
use crate::config::{Config, Connection};
use crate::error::{Error, Result};
use crate::handle::LidarHandle;
use crate::types::{Event, LedMode, WorkingMode};
use cxx::UniquePtr;
use std::time::Duration;

/// How long to back off when the SDK has no complete message buffered.
pub(crate) const IDLE_SLEEP: Duration = Duration::from_micros(500);

/// A connected Unitree L1 lidar.
///
/// ```no_run
/// use unitree_lidar_l1_rust::{Config, Event, Lidar};
///
/// let mut lidar = Lidar::open(Config::serial("/dev/ttyUSB0"))?;
/// for event in lidar.events() {
///     match event {
///         Event::Cloud(cloud) => println!("{} points", cloud.len()),
///         Event::Imu(imu) => println!("imu {:?}", imu.quaternion),
///     }
/// }
/// # Ok::<(), unitree_lidar_l1_rust::Error>(())
/// ```
///
/// When dropped, the lidar is put into standby.
pub struct Lidar {
    inner: UniquePtr<UnitreeLidarWrapper>,
    config: Config,
}

// SAFETY: the SDK reader holds no thread-local state. `Lidar` is only ever
// used through `&mut self`, so it is never accessed from two threads at once.
unsafe impl Send for Lidar {}

impl Lidar {
    /// Connects to the lidar and, unless [`Config::start_on_open`] is `false`,
    /// starts it (see [`Lidar::start`]).
    pub fn open(config: Config) -> Result<Self> {
        config.validate().map_err(Error::InvalidConfig)?;

        let mut inner = ffi::make_unitree_lidar_reader();
        let connected = match &config.connection {
            Connection::Serial { port, baudrate } => inner.pin_mut().initialize(
                config.cloud_scan_num,
                port,
                *baudrate,
                config.rotate_yaw_bias,
                config.range_scale,
                config.range_bias,
                config.range_max,
                config.range_min,
            ),
            Connection::Udp {
                lidar_ip,
                lidar_port,
                local_ip,
                local_port,
            } => inner.pin_mut().initialize_udp(
                config.cloud_scan_num,
                *lidar_port,
                lidar_ip,
                *local_port,
                local_ip,
                config.rotate_yaw_bias,
                config.range_scale,
                config.range_bias,
                config.range_max,
                config.range_min,
            ),
        };
        if !connected {
            return Err(Error::Connect(config.connection));
        }

        let mut lidar = Self { inner, config };
        if lidar.config.start_on_open {
            lidar.start();
        }
        Ok(lidar)
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Spins the lidar up by cycling standby -> normal. Blocks for about one second.
    pub fn start(&mut self) {
        self.set_working_mode(WorkingMode::Standby);
        std::thread::sleep(Duration::from_secs(1));
        self.set_working_mode(WorkingMode::Normal);
    }

    /// Stops the motor and measurements.
    pub fn standby(&mut self) {
        self.set_working_mode(WorkingMode::Standby);
    }

    pub fn set_working_mode(&mut self, mode: WorkingMode) {
        self.inner.pin_mut().set_lidar_working_mode(mode.into());
    }

    /// Resets the lidar.
    pub fn reset(&mut self) {
        self.inner.pin_mut().reset();
    }

    /// Plays one of the built-in LED ring animations.
    pub fn set_led_mode(&mut self, mode: LedMode) {
        self.inner.pin_mut().set_led_display_mode(mode.into());
    }

    /// Sets each of the 360 LEDs on the ring individually, one bit per LED.
    pub fn set_led_table(&mut self, table: &[u8; 45]) {
        self.inner.pin_mut().set_led_table(table);
    }

    /// Firmware version reported by the lidar.
    /// Empty until the lidar has sent its version message.
    pub fn firmware_version(&mut self) -> String {
        self.inner.pin_mut().get_version_of_firmware()
    }

    pub fn sdk_version(&mut self) -> String {
        self.inner.pin_mut().get_version_of_sdk()
    }

    /// One-way transmission delay between the lidar and this machine.
    pub fn time_delay(&mut self) -> Duration {
        Duration::from_micros(self.inner.pin_mut().get_time_delay() as u64)
    }

    /// Percentage of points removed because of dirt on the protective cover.
    pub fn dirty_percentage(&mut self) -> f32 {
        self.inner.pin_mut().get_dirty_percentage()
    }

    /// Makes one attempt to parse buffered data. Returns `None` right away
    /// if no point cloud or IMU reading is ready.
    pub fn try_next_event(&mut self) -> Option<Event> {
        match self.inner.pin_mut().run_parse() {
            MessageTypeRust::PointCloud => {
                let cloud: crate::PointCloud = self.inner.pin_mut().get_cloud().into();
                (!cloud.is_empty()).then_some(Event::Cloud(cloud))
            }
            MessageTypeRust::Imu => Some(Event::Imu(self.inner.pin_mut().get_imu().into())),
            _ => None,
        }
    }

    /// Blocks until the next point cloud or IMU reading arrives.
    pub fn next_event(&mut self) -> Event {
        loop {
            if let Some(event) = self.try_next_event() {
                return event;
            }
            std::thread::sleep(IDLE_SLEEP);
        }
    }

    /// Blocking iterator over events. It never ends on its own, so `break` out of it.
    pub fn events(&mut self) -> Events<'_> {
        Events { lidar: self }
    }

    /// Moves the lidar onto a background thread and returns a handle
    /// that receives its events.
    pub fn spawn(self) -> LidarHandle<Event> {
        LidarHandle::spawn(vec![(self, ())], |(), event| event)
    }

    /// Like [`Lidar::spawn`], but the handle can be `.await`ed and is a `Stream`.
    /// The returned handle works with any async runtime.
    #[cfg(feature = "async")]
    pub fn spawn_async(self) -> crate::AsyncLidarHandle<Event> {
        crate::AsyncLidarHandle::spawn(vec![(self, ())], |(), event| event)
    }
}

impl Drop for Lidar {
    fn drop(&mut self) {
        self.standby();
        if matches!(self.config.connection, Connection::Udp { .. }) {
            self.inner.pin_mut().close_udp();
        }
    }
}

/// Iterator returned by [`Lidar::events`].
pub struct Events<'a> {
    lidar: &'a mut Lidar,
}

impl Iterator for Events<'_> {
    type Item = Event;

    fn next(&mut self) -> Option<Event> {
        Some(self.lidar.next_event())
    }
}
