//! Rust wrapper for the Unitree L1 lidar SDK.
//!
//! ```no_run
//! use unitree_lidar_l1_rust::{Config, Event, Lidar};
//!
//! let mut lidar = Lidar::open(Config::serial("/dev/ttyUSB0").range(0.0, 50.0))?;
//!
//! for event in lidar.events() {
//!     match event {
//!         Event::Cloud(cloud) => println!("{} points", cloud.len()),
//!         Event::Imu(imu) => println!("imu {:?}", imu.quaternion),
//!     }
//! }
//! # Ok::<(), unitree_lidar_l1_rust::Error>(())
//! ```
//!
//! # Features
//!
//! - `async`: adds [`Lidar::spawn_async`] and `AsyncLidarHandle`, which work with any runtime.
//! - `nalgebra`: adds `nalgebra` conversions on [`Point`], [`PointCloud`] and [`Imu`],
//!   and re-exports [`nalgebra`].

mod bridge;
mod config;
mod error;
mod handle;
mod lidar;
mod types;

pub use config::{Config, Connection};
pub use error::{Error, Result};
#[cfg(feature = "async")]
pub use handle::AsyncLidarHandle;
pub use handle::{LidarGroup, LidarHandle};
pub use lidar::{Events, Lidar};
pub use types::{Event, Imu, LedMode, Point, PointCloud, WorkingMode};

#[cfg(feature = "async")]
pub use futures_core::Stream;
#[cfg(feature = "nalgebra")]
pub use nalgebra;
