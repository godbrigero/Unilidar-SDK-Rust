use unitree_lidar_l1_rust::{Config, Event, Lidar};

#[tokio::main]
async fn main() -> unitree_lidar_l1_rust::Result<()> {
    let port = std::env::args().nth(1).unwrap_or("/dev/ttyUSB0".into());
    let mut lidar = Lidar::open(Config::serial(port))?.spawn_async();

    while let Some(event) = lidar.recv().await {
        if let Event::Cloud(cloud) = event {
            println!("cloud #{}: {} points", cloud.id, cloud.len());
        }
    }
    Ok(())
}
