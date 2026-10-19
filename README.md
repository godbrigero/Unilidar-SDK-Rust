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
git clone https://github.com/godbrigero/Unilidar-SDK-Rust
cd unitree_lidar_l1_rust
cargo build
```

## How to use?

Everything you need is exported from the crate root.

```rust
use unitree_lidar_l1_rust::{Config, Event, Lidar};

let mut lidar = Lidar::open(Config::serial("/dev/ttyUSB0").range(0.0, 50.0))?;

for event in lidar.events() {
    match event {
        Event::Cloud(cloud) => println!("{} points at {}", cloud.len(), cloud.stamp),
        Event::Imu(imu) => println!("imu: {:?}", imu.quaternion),
    }
}
```

`Lidar::open` connects and spins the lidar up. Dropping the `Lidar` puts it back into standby.
For a UDP board, use `Config::udp()` (optionally with `.lidar_addr(..)` / `.local_addr(..)`).

### Background thread

```rust
let handle = Lidar::open(Config::serial("/dev/ttyUSB0"))?.spawn();

while let Some(event) = handle.recv() { /* ... */ }   // or handle.try_recv(), handle.recv_timeout(..)
let lidar = handle.stop();                           // get the lidar back
```

### Multiple lidars

```rust
use unitree_lidar_l1_rust::LidarGroup;

let lidars = LidarGroup::new()
    .with("front", Lidar::open(Config::serial("/dev/ttyUSB0"))?)
    .with("rear", Lidar::open(Config::serial("/dev/ttyUSB1"))?)
    .spawn();

for (name, event) in lidars {
    // ...
}
```

### Async (`features = ["async"]`)

Works with any runtime; the lidar still runs on its own thread.

```rust
let mut lidar = Lidar::open(Config::serial("/dev/ttyUSB0"))?.spawn_async();

while let Some(event) = lidar.recv().await { /* ... */ }
```

`AsyncLidarHandle` also implements `Stream` (re-exported as `unitree_lidar_l1_rust::Stream`).

### Features

- `async`: `Lidar::spawn_async` / `LidarGroup::spawn_async`
- `nalgebra`: `Point::to_vector3`, `PointCloud::to_vectors`, `Imu::orientation`, and re-exports `nalgebra`

## How to build?

```bash
cargo build
```
