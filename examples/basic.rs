use unitree_lidar_l1_rust::{Config, Event, Lidar};

fn main() -> unitree_lidar_l1_rust::Result<()> {
    let port = std::env::args().nth(1).unwrap_or("/dev/ttyUSB0".into());
    let mut lidar = Lidar::open(Config::serial(port).range(0.0, 50.0))?;
    println!("sdk {}", lidar.sdk_version());

    for event in lidar.events() {
        match event {
            Event::Cloud(cloud) => println!("cloud #{}: {} points", cloud.id, cloud.len()),
            Event::Imu(imu) => println!("imu: {:?}", imu.quaternion),
        }
    }
    Ok(())
}
