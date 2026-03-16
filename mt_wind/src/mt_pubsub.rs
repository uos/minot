use std::collections::HashMap;

use anyhow::anyhow;
use log::{debug, error, info, warn};
use mt_net::SensorTypeMapped;
use mt_pubsub::{Node, NodeConfig, Qos};
use mt_sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<Vec<mt_sea::WindData>>> {
    let kind = ShipKind::Wind(name.to_string());
    let ship =
        mt_sea::ship::NetworkShipImpl::init(kind.clone(), false, mt_sea::Qos::Reliable).await?;
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

    Ok(rx)
}

pub async fn run_dyn_wind(
    wind_name: &str,
    ready: tokio::sync::oneshot::Sender<()>,
) -> anyhow::Result<Option<()>> {
    let nh_name = wind_name.to_string() + "_pub";
    let nh = tokio::spawn(async move { Node::create(NodeConfig::new(nh_name)).await });
    let wind_name_owned = wind_name.to_owned();
    let wind_receiver = tokio::spawn(async move { wind(&wind_name_owned).await });

    let node = nh.await??;
    let mut wind_receiver = wind_receiver.await??;

    let mut cloud_publishers = HashMap::new();
    let mut odom_publishers = HashMap::new();
    let mut imu_publishers = HashMap::new();
    let mut any_type_warned = false;
    if ready.send(()).is_err() {
        warn!("mt_pubsub could not signal to be ready to handle requests");
    }
    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                SensorTypeMapped::Lidar(cloud_msg) => {
                    let mut existing_pubber = cloud_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = node
                            .create_publisher(data.topic.clone(), Qos::Reliable)
                            .await?;
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
                    debug!("published cloud");
                }
                SensorTypeMapped::Imu(imu_msg) => {
                    let mut existing_pubber = imu_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = node
                            .create_publisher(data.topic.clone(), Qos::Reliable)
                            .await?;
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
                    debug!("published imu");
                }
                SensorTypeMapped::Any(_) => {
                    if !any_type_warned {
                        warn!(
                            "Any-Types are not supported for mt_pubsub due to different message encodings, skipping."
                        );
                        any_type_warned = true;
                    }
                }
                SensorTypeMapped::Odometry(odom_msg) => {
                    let mut existing_pubber = odom_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = node
                            .create_publisher(data.topic.clone(), Qos::Reliable)
                            .await?;
                        odom_publishers.insert(data.topic.clone(), pubber);
                        existing_pubber = Some(
                            odom_publishers
                                .get(&data.topic)
                                .expect("Just inserted the line before"),
                        );
                    }
                    let pubber =
                        existing_pubber.expect("Should be inserted manually if not exists.");
                    pubber.publish(&odom_msg).await?;
                    debug!("published odometry");
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
    mt_sea::init_logging();

    let wind_name = get_env_or_default("wind_name", "turbine_mt_pubsub")?;

    let (tx, rx) = tokio::sync::oneshot::channel();

    tokio::spawn(async move {
        // dump ready answer
        _ = rx.await;
    });
    run_dyn_wind(&wind_name, tx).await?;

    Ok(())
}
