use std::collections::HashMap;

use anyhow::anyhow;
use log::{debug, error, info, warn};
use mt_net::SensorTypeMapped;
use mt_pubsub::{CoordMode, Node, NodeConfig, Qos};
use mt_sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<Vec<mt_sea::WindData>>> {
    let kind = ShipKind::Wind(name.to_string());
    let ship = std::sync::Arc::new(
        mt_sea::ship::NetworkShipImpl::init(
            kind.clone(),
            false,
            mt_sea::Qos::Reliable,
            mt_sea::NodeOptions::default(),
        )
        .await?,
    );
    ship.spawn_heartbeat();
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

/// Env variable selecting how the turbine's publishers reach subscribers:
/// `peer` (default), `autostart`, `start` or `external`.
pub const COORD_MODE_ENV: &str = "wind_ratpub_coord_mode";

/// Coordination mode of the turbine's publishers.
///
/// Coordinated and peer topics are separate: a coordinated subscriber never
/// sees a peer publisher, so choose the mode the subscribers use. Replay
/// control stays on the coordinator in every mode.
pub fn publisher_coord_mode() -> anyhow::Result<CoordMode> {
    let value = crate::get_env_or_default(COORD_MODE_ENV, "peer")?;
    match value.to_ascii_lowercase().replace(['-', '_'], "").as_str() {
        "autostart" => Ok(CoordMode::AutoStart),
        "start" => Ok(CoordMode::Start),
        "external" => Ok(CoordMode::External),
        "peer" => Ok(CoordMode::Peer),
        _ => Err(anyhow!(
            "Invalid {COORD_MODE_ENV} '{value}', expected autostart, start, external or peer"
        )),
    }
}

/// Whether the turbine's publisher node registers with the coordinator, so the
/// coordinator must wait for it among the expected clients.
pub fn publisher_registers() -> anyhow::Result<bool> {
    Ok(!matches!(publisher_coord_mode()?, CoordMode::Peer))
}

pub async fn run_dyn_wind(
    wind_name: &str,
    ready: tokio::sync::oneshot::Sender<()>,
) -> anyhow::Result<Option<()>> {
    let nh_name = wind_name.to_string() + "_pub";
    let coord_mode = publisher_coord_mode()?;
    let nh = tokio::spawn(async move {
        Node::create(NodeConfig::new(nh_name).coord_mode(coord_mode)).await
    });
    let wind_name_owned = wind_name.to_owned();
    let wind_receiver = tokio::spawn(async move { wind(&wind_name_owned).await });

    let node = nh.await??;
    let mut wind_receiver = wind_receiver.await??;

    let mut image_publishers = HashMap::new();
    let mut compressed_image_publishers = HashMap::new();
    let mut camera_info_publishers = HashMap::new();
    let mut cloud_publishers = HashMap::new();
    let mut odom_publishers = HashMap::new();
    let mut imu_publishers = HashMap::new();
    let mut tf_publishers = HashMap::new();
    let mut any_type_warned = false;
    let mut clock_warned = false;
    if ready.send(()).is_err() {
        warn!("mt_pubsub could not signal to be ready to handle requests");
    }
    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                SensorTypeMapped::Image(msg) => {
                    if !image_publishers.contains_key(&data.topic) {
                        let qos = if matches!(&data.qos, Some(mt_net::Qos::Custom(profile)) if profile.durability == "transient_local")
                        {
                            Qos::Reliable.transient_local()
                        } else {
                            Qos::Reliable
                        };
                        let publisher = node.create_publisher(data.topic.clone(), qos).await?;
                        image_publishers.insert(data.topic.clone(), publisher);
                    }
                    image_publishers
                        .get(&data.topic)
                        .expect("Publisher exists")
                        .publish(&msg)
                        .await?;
                }
                SensorTypeMapped::CompressedImage(msg) => {
                    if !compressed_image_publishers.contains_key(&data.topic) {
                        let qos = if matches!(&data.qos, Some(mt_net::Qos::Custom(profile)) if profile.durability == "transient_local")
                        {
                            Qos::Reliable.transient_local()
                        } else {
                            Qos::Reliable
                        };
                        let publisher = node.create_publisher(data.topic.clone(), qos).await?;
                        compressed_image_publishers.insert(data.topic.clone(), publisher);
                    }
                    compressed_image_publishers
                        .get(&data.topic)
                        .expect("Publisher exists")
                        .publish(&msg)
                        .await?;
                }
                SensorTypeMapped::CameraInfo(msg) => {
                    if !camera_info_publishers.contains_key(&data.topic) {
                        let qos = if matches!(&data.qos, Some(mt_net::Qos::Custom(profile)) if profile.durability == "transient_local")
                        {
                            Qos::Reliable.transient_local()
                        } else {
                            Qos::Reliable
                        };
                        let publisher = node.create_publisher(data.topic.clone(), qos).await?;
                        camera_info_publishers.insert(data.topic.clone(), publisher);
                    }
                    camera_info_publishers
                        .get(&data.topic)
                        .expect("Publisher exists")
                        .publish(&msg)
                        .await?;
                }
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
                SensorTypeMapped::Tf(tf_msg) => {
                    let mut existing_pubber = tf_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        // Keep latched topics like /tf_static latched for late subscribers.
                        let transient_local = matches!(
                            &data.qos,
                            Some(mt_net::Qos::Custom(profile)) if profile.durability == "transient_local"
                        );
                        let qos = if transient_local {
                            Qos::Reliable.transient_local()
                        } else {
                            Qos::Reliable
                        };
                        let pubber = node.create_publisher(data.topic.clone(), qos).await?;
                        tf_publishers.insert(data.topic.clone(), pubber);
                        existing_pubber = Some(
                            tf_publishers
                                .get(&data.topic)
                                .expect("Just inserted the line before"),
                        );
                    }
                    let pubber =
                        existing_pubber.expect("Should be inserted manually if not exists.");
                    pubber.publish(&tf_msg).await?;
                    debug!("published tf");
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
                SensorTypeMapped::Clock(_) => {
                    if !clock_warned {
                        warn!("Synthetic /clock is only published by ROS winds, skipping.");
                        clock_warned = true;
                    }
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
