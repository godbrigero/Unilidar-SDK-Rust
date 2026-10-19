use crate::config::Connection;
use std::fmt;

pub type Result<T, E = Error> = std::result::Result<T, E>;

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The serial port / UDP socket could not be opened.
    Connect(Connection),
    /// The [`Config`](crate::Config) contains invalid values.
    InvalidConfig(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Connect(Connection::Serial { port, baudrate }) => {
                write!(f, "failed to open lidar on serial port {port} ({baudrate} baud)")
            }
            Error::Connect(Connection::Udp {
                lidar_ip,
                lidar_port,
                local_ip,
                local_port,
            }) => write!(
                f,
                "failed to open lidar over UDP (lidar {lidar_ip}:{lidar_port}, local {local_ip}:{local_port})"
            ),
            Error::InvalidConfig(reason) => write!(f, "invalid lidar config: {reason}"),
        }
    }
}

impl std::error::Error for Error {}
