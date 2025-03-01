use std::collections::HashMap;

use log::{error, info};
use ros_pointcloud2::{points::PointXYZ, PointCloud2Msg};
use sea::{coordinator::CoordinatorImpl, ImuMsg};
use tokio::sync::{broadcast, oneshot};
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
// hashmap[string] => ID
// ID => VariableRule

use sea::Coordinator;

#[derive(Debug)]
enum CoordinatorTask {
    GetVariableChannel {
        ship: String,
        answer: oneshot::Sender<broadcast::Receiver<String>>,
    },
    SendWind {
        ship_name: String,
        data: sea::WindData,
    },
    SendRatAction {
        ship_name: String,
        data: sea::ActionPlan,
    },
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    // TODO config logic

    let mut pairs = HashMap::new();
    pairs.insert(
        "var1".to_string(),
        (
            sea::VariableHuman {
                ship: "testRat1".to_string(),
                strategy: Some(sea::ActionPlan::Shoot {
                    target: vec!["testRat2".to_string()],
                }),
            },
            sea::VariableHuman {
                ship: "testRat2".to_string(),
                strategy: Some(sea::ActionPlan::Catch {
                    source: "testRat1".to_string(),
                }),
            },
        ),
    );

    // let sea = sea::net::Sea::init(None).await;

    // TODO collect all available rats from network.

    // TODO collect all winds.

    // TODO parse config/ratlang and see if all mentioned rats are registered
    // err and exit if not.
    // On exit: disconnect all members of the network.
    // only use the rats that are mentioned. filter out the rest.
    // Winds are not mapped to rats, so they won't be filtered.

    // TODO variables come dynamically from the network. We prepare them from the
    // rat config and ignore the rest. The known ones are parsed to pairs.

    // TODO a function must build this one and check that the names are the same
    // if a variable is asking what to do but it is not part of the structure,
    // answer with a "keep".

    let pairs = std::sync::Arc::new(pairs);

    // TODO polulate Network by rats and winds

    // TODO populate rats variables completely so we can listen on all separately
    let network = Network::new();

    // Task for centralized coordinator work
    let (coord_tx, mut coord_rx) = tokio::sync::mpsc::unbounded_channel::<CoordinatorTask>();
    tokio::spawn(async move {
        let coordinator = CoordinatorImpl::new(None).await;

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
                    match coordinator.blow_wind(ship_name.clone(), data).await {
                        Err(e) => {
                            error!("Error while blowing wind to {}: {}", ship_name, e);
                            std::process::exit(1);
                        }
                        _ => (),
                    }
                }
                CoordinatorTask::SendRatAction { ship_name, data } => {
                    match coordinator.rat_action_send(ship_name.clone(), data).await {
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
                ship: rat.name.clone(),
                answer: answer_tx,
            })
            .unwrap();
        let mut queue = answer_rx.await.unwrap();

        let rat_coord_tx = coord_tx.clone();
        let rat_pairs = pairs.clone();
        tokio::spawn({
            async move {
                while let Ok(variable) = queue.recv().await {
                    let action = sea::get_strategy(&rat_pairs, &rat.name, variable);
                    rat_coord_tx
                        .send(CoordinatorTask::SendRatAction {
                            ship_name: rat.name.clone(),
                            data: action,
                        })
                        .unwrap();
                }
            }
        });
    }

    // Wind Sender Task
    let (wind_tx, mut wind_rx) = tokio::sync::mpsc::unbounded_channel::<sea::WindData>();
    let wind_coord_tx = coord_tx.clone();
    tokio::spawn(async move {
        while let Some(data) = wind_rx.recv().await {
            for wind in network.winds.iter() {
                wind_coord_tx
                    .send(CoordinatorTask::SendWind {
                        ship_name: wind.name.clone(),
                        data: data.clone(),
                    })
                    .unwrap();
            }
        }
    });

    // The bag reader is a different task that can be notified with a channel what to do
    //let bagfile = "bagfile";
    // TODO bag reader task, create random data for now, bagfile later!

    let rand_imu = ImuMsg::default();
    let imu_data = sea::WindData::Imu(rand_imu);
    wind_tx.send(imu_data).unwrap();

    let rand_points = vec![PointXYZ::new(1.0, 1.0, 1.0)];
    let rand_points = PointCloud2Msg::try_from_vec(rand_points).unwrap();
    let lidar_data = sea::WindData::Pointcloud(rand_points);
    wind_tx.send(lidar_data).unwrap();

    info!("Listening...");
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            info!("Ctrl-C received. Shutting down...");
        }
    }

    Ok(())
}
