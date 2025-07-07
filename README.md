# Unitree Lidar SDK

## What is this?

As you know, the official SDK is not available for rust users since it it written in C++. This project is a wrapper for the official SDK that allows you to use the SDK in rust.

## **Supported platforms**

- Linux x86_64
- Linux aarch64

## Installation

### From crates.io

```bash
cargo add unitree-lidar-l1-rust
```

The static libraries are included in the published crate, so no additional setup is required!

### From Source

```bash
git clone https://github.com/yourusername/unitree_lidar_l1_rust
cd unitree_lidar_l1_rust
cargo build
```

## How to use?

```rust
let mut reader = LidarReader::new("/dev/ttyUSB0", 0.0, 50.0).unwrap();
reader.start_lidar();
let mut stream = reader.into_stream();

while let Some(result) = stream.next() {
    match result {
        LidarResult::PointCloud(points) => {
            println!("Point cloud: {:?}", points);
        }
        LidarResult::ImuReading(imu) => {
            println!("IMU reading: {:?}", imu);
        }
    }
}
```

## How to build?

```bash
cargo build
```
