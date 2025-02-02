use log::error;
use nalgebra::*;
use ros_pointcloud2::{points::PointXYZ, CloudDimensions, PointCloud2Msg, PointCloud2MsgBuilder};
use sea::ImuMsg;
use serde::{Deserialize, Serialize};

mod network;

struct Network {
    rats: Vec<network::Rat>,
    winds: Vec<network::Wind>,
    coordinator: Box<dyn sea::Coordinator>,
}

impl Network {
    fn new() -> Self {
        Network {
            rats: Vec::new(),
            winds: Vec::new(),
            coordinator: Box::new(sea::coordinator::CoordinatorImpl::new()),
        }
    }
}

pub type RatId = u32;

#[derive(Debug, Copy, Clone)]
enum VariableOptions {
    NoOp,
    MoveTo(RatId),
    ReceiveFrom(RatId),
}

#[derive(Debug, Copy, Clone)]
enum BagReaderTask {
    LidarFrame(u32),
    OdomFrame(u32),
    MixedWhile(f64), // Time in seconds
                     // More here like "mixed until n lidar" or "random out of sync" etc
}

#[derive(Debug, Copy, Clone)]
enum VariableRule {}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Hello, world!");
    // TODO Initialize Sea Network - detect all ships

    // TODO polulate Network by rats and winds

    // TODO populate rats variables completely so we can listen on all separately
    let network = Network::new();

    // Spawn a task for each rat to listen when variables come in
    for rat in network.rats.iter() {
        let mut queue = network
            .coordinator
            .rat_action_request_queue(rat.network_id)
            .await?;
        tokio::spawn(async move {
            while let Some(variable) = queue.recv().await {
                // TODO handle variable based on rules
            }
        });
    }

    // Winds get posted by us. For demo, just push 1 second of a bag file to all winds
    let duration = std::time::Duration::from_secs(1);

    // The bag reader is a different task that can be notified with a channel what to do
    let bagfile = "bagfile";
    tokio::spawn(async move {
        // TODO bag reader task, create random data for now, bagfile later!
        let rand_imu = ImuMsg::default();
        let imu_data = sea::WindData::Imu(rand_imu);

        for wind in network.winds.iter() {
            if let Err(err) = network
                .coordinator
                .wind_set_sail(wind.network_id, imu_data.clone())
                .await
            {
                error!("Error while setting sail: {}", err); // TODO maybe cancel everything here because state is not sync anymore
            }
        }

        let rand_points = vec![PointXYZ::new(1.0, 1.0, 1.0)];
        let rand_points = PointCloud2Msg::try_from_vec(rand_points).unwrap();
        let lidar_data = sea::WindData::Pointcloud(rand_points);
    });

    Ok(())
}
