use std::collections::HashMap;

use anyhow::anyhow;
use log::{error, info};
use ratpubsub::Node;
use sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<Vec<sea::WindData>>> {
    let kind = ShipKind::Wind(name.to_string());
    let ship = sea::ship::NetworkShipImpl::init(kind.clone(), None).await?;
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

pub async fn run_dyn_wind(wind_name: &str) -> anyhow::Result<Option<()>> {
    let node = Node::create(wind_name.to_owned()).await?;
    let mut wind_receiver = wind(&wind_name).await?;

    let mut cloud_publishers = HashMap::new();
    let mut imu_publishers = HashMap::new();
    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                sea::SensorTypeMapped::Lidar(cloud_msg) => {
                    let mut existing_pubber = cloud_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = node.create_publisher(data.topic.clone()).await?;
                        cloud_publishers.insert(data.topic.clone(), pubber);
                        existing_pubber = Some(
                            cloud_publishers
                                .get(&data.topic)
                                .expect("Just inserted the line before"),
                        );
                    }
                    let pubber =
                        existing_pubber.expect("Should be inserted manually if not exists.");
                    pubber.publish(&cloud_msg).await?;
                }
                sea::SensorTypeMapped::Imu(imu_msg) => {
                    let mut existing_pubber = imu_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = node.create_publisher(data.topic.clone()).await?;
                        imu_publishers.insert(data.topic.clone(), pubber);
                        existing_pubber = Some(
                            imu_publishers
                                .get(&data.topic)
                                .expect("Just inserted the line before"),
                        );
                    }
                    let pubber =
                        existing_pubber.expect("Should be inserted manually if not exists.");
                    pubber.publish(&imu_msg).await?;
                }
                sea::SensorTypeMapped::Any(_) => {
                    error!(
                        "Any-Types are not supported for ratpubsub due to different message encodings."
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

    let wind_name = get_env_or_default("wind_name", "turbine_ratpubsub")?;

    run_dyn_wind(&wind_name).await?;

    Ok(())
}
