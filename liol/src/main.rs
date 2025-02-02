use log::error;
use nalgebra::*;
use ros_pointcloud2::{points::PointXYZ, CloudDimensions, PointCloud2Msg, PointCloud2MsgBuilder};
use sea::{coordinator::CoordinatorImpl, Action, ImuMsg, ShipName};
use serde::{Deserialize, Serialize};
use std::{
    future::IntoFuture,
    sync::{Arc, Mutex},
};
use tokio::sync::{
    broadcast::Sender,
    mpsc::{UnboundedReceiver, UnboundedSender},
};
use tokio::sync::{mpsc, oneshot};
use tokio::task;
mod network;

struct Network {
    rats: Vec<network::Rat>,
    winds: Vec<network::Wind>,
}

impl Network {
    fn new() -> Self {
        Network {
            rats: Vec::new(),
            winds: Vec::new(),
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

// All possible variables that should be acted on must be known beforehand.
// They get an ID or get generated into a type? TODO
// ID => VariableRule
// All

#[derive(Debug, Copy, Clone)]
enum VariableRule {}

use sea::Coordinator;

#[derive(Debug)]
enum CoordinatorTask {
    GetVariableChannel {
        ship: ShipName,
        answer: oneshot::Sender<mpsc::Receiver<String>>,
    },
    SendWind {
        ship_name: ShipName,
        data: sea::WindData,
    },
    SendRatAction {
        ship_name: ShipName,
        data: sea::Action,
    },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // TODO parse from somewhere later, just temporary hardcoded
    let variables = std::sync::Arc::new(vec!["bla".to_string()]);

    // TODO Initialize Sea Network - detect all ships

    // TODO polulate Network by rats and winds

    // TODO populate rats variables completely so we can listen on all separately
    let network = Network::new();

    // Task for centralized coordinator work
    let (coord_tx, mut coord_rx) = tokio::sync::mpsc::unbounded_channel::<CoordinatorTask>();
    tokio::spawn(async move {
        let coordinator = CoordinatorImpl::new();

        while let Some(task) = coord_rx.recv().await {
            match task {
                CoordinatorTask::GetVariableChannel { ship, answer } => {
                    let queue = match coordinator.rat_action_request_queue(ship).await {
                        Err(e) => {
                            error!("Error getting rat queue: {}", e);
                            std::process::exit(1);
                        }
                        Ok(queue) => queue,
                    };

                    answer.send(queue).unwrap();
                }
                CoordinatorTask::SendWind { ship_name, data } => {
                    match coordinator.blow_wind(ship_name, data).await {
                        Err(e) => {
                            error!("Error while blowing wind to {}: {}", ship_name, e);
                            std::process::exit(1);
                        }
                        _ => (),
                    }
                }
                CoordinatorTask::SendRatAction { ship_name, data } => {
                    match coordinator.rat_action_send(ship_name, data).await {
                        Err(e) => {
                            error!("Error while sending action to Rat {}: {}", ship_name, e);
                            std::process::exit(1);
                        }
                        Ok(_) => (),
                    }
                }
            }
        }
    });

    // Spawn a task for each rat to listen when variables come in
    for rat in network.rats.into_iter() {
        let (answer_tx, answer_rx) = oneshot::channel();
        coord_tx
            .send(CoordinatorTask::GetVariableChannel {
                ship: rat.network_id,
                answer: answer_tx,
            })
            .unwrap();
        let mut queue = answer_rx.await.unwrap();

        let var_clone = variables.clone();
        let rat_coord_tx = coord_tx.clone();
        tokio::spawn({
            async move {
                while let Some(variable) = queue.recv().await {
                    // TODO handle variable based on rules
                    let action = if !var_clone.contains(&variable) {
                        sea::Action::Sail
                    } else {
                        sea::Action::Sail // TODO actually evaluate rules
                    };

                    rat_coord_tx
                        .send(CoordinatorTask::SendRatAction {
                            ship_name: rat.network_id,
                            data: action,
                        })
                        .unwrap();
                }
            }
        });
    }

    // Winds get posted by us. For demo, just push 1 second of a bag file to all winds
    //let duration = std::time::Duration::from_secs(1);

    // The bag reader is a different task that can be notified with a channel what to do
    //let bagfile = "bagfile";
    // TODO bag reader task, create random data for now, bagfile later!

    // Wind Sender Task
    let (wind_tx, mut wind_rx) = tokio::sync::mpsc::unbounded_channel::<sea::WindData>();
    let wind_coord_tx = coord_tx.clone();
    tokio::spawn(async move {
        while let Some(data) = wind_rx.recv().await {
            for wind in network.winds.iter() {
                wind_coord_tx
                    .send(CoordinatorTask::SendWind {
                        ship_name: wind.network_id,
                        data: data.clone(),
                    })
                    .unwrap();
            }
        }
    });

    let rand_imu = ImuMsg::default();
    let imu_data = sea::WindData::Imu(rand_imu);
    wind_tx.send(imu_data).unwrap();

    let rand_points = vec![PointXYZ::new(1.0, 1.0, 1.0)];
    let rand_points = PointCloud2Msg::try_from_vec(rand_points).unwrap();
    let lidar_data = sea::WindData::Pointcloud(rand_points);
    wind_tx.send(lidar_data).unwrap();

    Ok(())
}
