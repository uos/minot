use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use anyhow::anyhow;
use log::{error, info};
use ratpub::{Node, Publisher};
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{Imu, PointCloud2};
use sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<Vec<sea::WindData>>> {
    let kind = ShipKind::Wind(name.to_string());
    let ship = sea::ship::NetworkShipImpl::init(kind.clone(), None, false).await?;
    info!("Wind initialized with ship {:?}", kind);

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();

    tokio::spawn(async move {
        loop {
            match ship.wait_for_wind().await {
                Ok(wind_data) => {
                    tx.send(wind_data).unwrap();
                }
                Err(e) => {
                    error!("{}", e);
                    continue;
                }
            }
        }
    });

    tokio::task::yield_now().await;

    Ok(rx)
}

// TODO currently only supports one topic because we can not send a publisher and we need that for hashmap
pub async fn run_dyn_wind(wind_name: &str) -> anyhow::Result<Option<()>> {
    let node = Node::create(wind_name.to_owned()).await?;
    let mut wind_receiver = wind(&wind_name).await?;

    let mut cloud_pub = None;
    let mut imu_pub = None;

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                sea::SensorTypeMapped::Lidar(cloud_msg) => {
                    let existing = match cloud_pub.as_ref() {
                        None => {
                            let pubber = node
                                .create_publisher::<PointCloud2>(data.topic.clone())
                                .await
                                .unwrap();

                            cloud_pub = Some(pubber);
                            cloud_pub.as_ref().unwrap()
                        }
                        Some(existing) => &existing,
                    };

                    existing.publish(&cloud_msg).await.unwrap();
                }
                sea::SensorTypeMapped::Imu(imu_msg) => {
                    let existing = match imu_pub.as_ref() {
                        None => {
                            let pubber = node
                                .create_publisher::<Imu>(data.topic.clone())
                                .await
                                .unwrap();

                            imu_pub = Some(pubber);
                            imu_pub.as_ref().unwrap()
                        }
                        Some(existing) => &existing,
                    };

                    existing.publish(&imu_msg).await.unwrap();
                }
                sea::SensorTypeMapped::Any(_) => {
                    error!(
                        "Any-Types are not supported for ratpub due to different message encodings."
                    )
                }
            }
        }
    }

    Ok(Some(()))
}

// TODO share in module so both submodules can use it
pub fn get_env_or_default(key: &str, default: &str) -> anyhow::Result<String> {
    match std::env::var(key) {
        Ok(name) => Ok(name),
        Err(e) => match e {
            std::env::VarError::NotPresent => Ok(default.to_owned()),
            std::env::VarError::NotUnicode(_os_string) => Err(anyhow!(
                "Could not fetch env variable because it is not unicode"
            )),
        },
    }
}

#[tokio::main]
#[allow(dead_code)]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let wind_name = get_env_or_default("wind_name", "turbine_ratpub")?;

    run_dyn_wind(&wind_name).await?;

    Ok(())
}
