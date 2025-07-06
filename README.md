# Unitree Lidar SDK

## What is this?

As you know, the official SDK is not available for rust users since it it written in C++. This project is a wrapper for the official SDK that allows you to use the SDK in rust.

## **Supported platforms**

- Linux x86_64
- Linux aarch64

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
