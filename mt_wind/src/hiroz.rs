use std::collections::{HashMap, HashSet};
use std::num::NonZeroUsize;
use std::str::FromStr;

use anyhow::anyhow;
use hiroz::dynamic::{DynamicMessage, DynamicSerdeCdrSerdes};
use hiroz::entity::{TypeHash, TypeInfo};
use hiroz::msg::NativeCdrSerdes;
use hiroz::pubsub::ZPub;
use hiroz::{
    Builder,
    context::ZContextBuilder,
    qos::{QosDurability, QosDuration, QosHistory, QosLiveliness, QosProfile, QosReliability},
};
use hiroz_msgs::{
    builtin_interfaces::Time,
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
    },
    nav_msgs::Odometry,
    rosgraph_msgs::Clock,
    sensor_msgs::{Imu, PointCloud2, PointField},
    std_msgs::Header,
};
use log::{error, info, warn};
use mt_bagread::qos::{
    RmwQosDurabilityPolicy, RmwQosHistoryPolicy, RmwQosLivelinessPolicy, RmwQosReliabilityPolicy,
};
use mt_sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

type HirozPub<T> = ZPub<T, NativeCdrSerdes<T>>;

fn type_hash_or_zero(
    topic: &str,
    msg_type: &str,
    type_description_hash: Option<&str>,
    malformed_hash_warned: &mut HashSet<(String, String)>,
) -> TypeHash {
    let Some(hash) = type_description_hash else {
        return TypeHash::zero();
    };

    TypeHash::from_rihs_string(hash).unwrap_or_else(|| {
        let key = (topic.to_string(), msg_type.to_string());
        if malformed_hash_warned.insert(key) {
            warn!(
                "Malformed type_description_hash '{}' for topic {} with type {}, using zero type hash",
                hash, topic, msg_type
            );
        }
        TypeHash::zero()
    })
}

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

#[derive(Clone, Debug, Default)]
pub enum Qos {
    Sensor,
    #[default]
    SystemDefault,
    Custom(mt_net::QosProfile),
}

impl From<mt_net::Qos> for Qos {
    fn from(value: mt_net::Qos) -> Self {
        match value {
            mt_net::Qos::Sensor => Qos::Sensor,
            mt_net::Qos::SystemDefault => Qos::SystemDefault,
            mt_net::Qos::Custom(qos_profile) => Qos::Custom(qos_profile),
        }
    }
}

fn hiroz_qos(value: Qos) -> anyhow::Result<Option<QosProfile>> {
    match value {
        Qos::Sensor => Ok(Some(QosProfile {
            reliability: QosReliability::BestEffort,
            durability: QosDurability::Volatile,
            history: QosHistory::KeepLast(NonZeroUsize::new(5).unwrap()),
            ..Default::default()
        })),
        Qos::SystemDefault => Ok(None),
        Qos::Custom(q) => Ok(Some(QosProfile {
            reliability: match RmwQosReliabilityPolicy::from_str(&q.reliability)? {
                RmwQosReliabilityPolicy::Reliable => QosReliability::Reliable,
                RmwQosReliabilityPolicy::BestEffort => QosReliability::BestEffort,
                RmwQosReliabilityPolicy::SystemDefault
                | RmwQosReliabilityPolicy::Unknown
                | RmwQosReliabilityPolicy::BestAvailable => QosReliability::default(),
            },
            durability: match RmwQosDurabilityPolicy::from_str(&q.durability)? {
                RmwQosDurabilityPolicy::TransientLocal => QosDurability::TransientLocal,
                RmwQosDurabilityPolicy::Volatile => QosDurability::Volatile,
                RmwQosDurabilityPolicy::SystemDefault
                | RmwQosDurabilityPolicy::Unknown
                | RmwQosDurabilityPolicy::BestAvailable => QosDurability::default(),
            },
            history: match RmwQosHistoryPolicy::from_str(&q.history)? {
                RmwQosHistoryPolicy::KeepLast
                | RmwQosHistoryPolicy::SystemDefault
                | RmwQosHistoryPolicy::Unknown => QosHistory::from_depth(q.depth.max(0) as usize),
                RmwQosHistoryPolicy::KeepAll => QosHistory::KeepAll,
            },
            deadline: qos_duration(q.deadline.sec, q.deadline.nsec),
            lifespan: qos_duration(q.lifespan.sec, q.lifespan.nsec),
            liveliness: match RmwQosLivelinessPolicy::from_str(&q.liveliness)? {
                RmwQosLivelinessPolicy::Automatic => QosLiveliness::Automatic,
                RmwQosLivelinessPolicy::ManualByNode => QosLiveliness::ManualByNode,
                RmwQosLivelinessPolicy::ManualByTopic => QosLiveliness::ManualByTopic,
                RmwQosLivelinessPolicy::SystemDefault
                | RmwQosLivelinessPolicy::Unknown
                | RmwQosLivelinessPolicy::BestAvailable => QosLiveliness::default(),
            },
            liveliness_lease_duration: qos_duration(
                q.liveliness_lease_duration.sec,
                q.liveliness_lease_duration.nsec,
            ),
        })),
    }
}

fn qos_duration(sec: u64, nsec: u64) -> QosDuration {
    QosDuration { sec, nsec }
}

fn env_bool_or_default(key: &str, default: bool) -> anyhow::Result<bool> {
    let value = match std::env::var(key) {
        Ok(value) => value,
        Err(std::env::VarError::NotPresent) => return Ok(default),
        Err(std::env::VarError::NotUnicode(_)) => {
            return Err(anyhow!("{key} is not valid unicode"));
        }
    };

    match value.to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Ok(true),
        "0" | "false" | "no" | "off" => Ok(false),
        _ => Err(anyhow!(
            "Invalid boolean value '{}' for {}; expected true/false",
            value,
            key
        )),
    }
}

fn env_usize_or_default(key: &str, default: usize) -> anyhow::Result<usize> {
    let value = match std::env::var(key) {
        Ok(value) => value,
        Err(std::env::VarError::NotPresent) => return Ok(default),
        Err(std::env::VarError::NotUnicode(_)) => {
            return Err(anyhow!("{key} is not valid unicode"));
        }
    };

    value
        .parse()
        .map_err(|e| anyhow!("Invalid usize value '{}' for {}: {}", value, key, e))
}

fn hiroz_shm_pool_size() -> anyhow::Result<usize> {
    match std::env::var("HIROZ_SHM_POOL_SIZE") {
        Ok(value) => value.parse().map_err(|e| {
            anyhow!(
                "Invalid usize value '{}' for HIROZ_SHM_POOL_SIZE: {}",
                value,
                e
            )
        }),
        Err(std::env::VarError::NotPresent) => {
            env_usize_or_default("ZENOH_SHM_ALLOC_SIZE", hiroz::shm::DEFAULT_SHM_POOL_SIZE)
        }
        Err(std::env::VarError::NotUnicode(_)) => {
            Err(anyhow!("HIROZ_SHM_POOL_SIZE is not valid unicode"))
        }
    }
}

fn hiroz_context_builder(
    domain_id: usize,
    mode: String,
    endpoint: String,
) -> anyhow::Result<ZContextBuilder> {
    let base = || {
        ZContextBuilder::default()
            .with_domain_id(domain_id)
            .with_mode(mode.clone())
            .with_connect_endpoints([endpoint.clone()])
    };

    if !env_bool_or_default("HIROZ_SHM", true)? {
        return Ok(base()
            .with_json("transport/shared_memory/enabled", false)
            .with_json(
                "transport/shared_memory/transport_optimization/enabled",
                false,
            ));
    }

    let pool_size = hiroz_shm_pool_size()?;
    let threshold = env_usize_or_default("HIROZ_SHM_THRESHOLD", hiroz::shm::DEFAULT_SHM_THRESHOLD)?;

    match base().with_shm_pool_size(pool_size) {
        Ok(builder) => Ok(builder.with_shm_threshold(threshold)),
        Err(error) => {
            warn!(
                "Hiroz SHM initialization failed: {}; continuing without SHM",
                error
            );
            Ok(base()
                .with_json("transport/shared_memory/enabled", false)
                .with_json(
                    "transport/shared_memory/transport_optimization/enabled",
                    false,
                ))
        }
    }
}

fn hiroz_domain_id() -> anyhow::Result<usize> {
    let value = match std::env::var("HIROZ_DOMAIN_ID") {
        Ok(value) => value,
        Err(std::env::VarError::NotPresent) => match std::env::var("ROS_DOMAIN_ID") {
            Ok(value) => value,
            Err(std::env::VarError::NotPresent) => return Ok(0),
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(anyhow!("ROS_DOMAIN_ID is not valid unicode"));
            }
        },
        Err(std::env::VarError::NotUnicode(_)) => {
            return Err(anyhow!("HIROZ_DOMAIN_ID is not valid unicode"));
        }
    };

    value
        .parse()
        .map_err(|e| anyhow!("Invalid Hiroz domain id '{}': {}", value, e))
}

pub async fn run_dyn_wind(
    wind_name: &str,
    ready: tokio::sync::oneshot::Sender<()>,
) -> anyhow::Result<()> {
    let mode = get_env_or_default("HIROZ_MODE", "client")?;
    let endpoint = get_env_or_default("HIROZ_ENDPOINT", "tcp/127.0.0.1:7447")?;
    let domain_id = hiroz_domain_id()?;
    let ctx = hiroz_context_builder(domain_id, mode, endpoint)?
        .build()
        .map_err(|e| anyhow!("{e}"))?;
    let node_name = wind_name.to_owned() + "_node";
    let node = ctx
        .create_node(&node_name)
        .build()
        .map_err(|e| anyhow!("{e}"))?;

    let mut cloud_publishers: HashMap<(String, String), HirozPub<PointCloud2>> = HashMap::new();
    let mut imu_publishers: HashMap<(String, String), HirozPub<Imu>> = HashMap::new();
    let mut odom_publishers: HashMap<(String, String), HirozPub<Odometry>> = HashMap::new();
    let mut clock_publishers: HashMap<(String, String), HirozPub<Clock>> = HashMap::new();
    let mut any_publishers: HashMap<(String, String, Option<String>), hiroz::dynamic::DynPub> =
        HashMap::new();
    let mut missing_qos_warned: HashSet<(String, String)> = HashSet::new();
    let mut malformed_hash_warned: HashSet<(String, String)> = HashSet::new();

    let mut wind_receiver = wind(wind_name).await?;

    if let Err(_) = ready.send(()) {
        warn!("hiroz wind could not signal to be ready to handle requests");
    }

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            let publisher_key = (data.topic.clone(), data.msg_type.clone());
            let qos = match data.qos {
                Some(qos) => hiroz_qos(Qos::from(qos))?,
                None => {
                    if missing_qos_warned.insert(publisher_key.clone()) {
                        warn!(
                            "Received message without QOS for topic {} with type {}, choosing Hiroz default",
                            data.topic, data.msg_type
                        );
                    }
                    None
                }
            };

            match data.data {
                mt_net::SensorTypeMapped::Lidar(cloud_msg) => {
                    if !cloud_publishers.contains_key(&publisher_key) {
                        let mut builder = node.create_pub::<PointCloud2>(&data.topic);
                        if let Some(qos) = qos {
                            builder = builder.with_qos(qos);
                        }
                        cloud_publishers.insert(
                            publisher_key.clone(),
                            builder.build().map_err(|e| anyhow!("{e}"))?,
                        );
                    }
                    let pubber = cloud_publishers
                        .get(&publisher_key)
                        .expect("Publisher exists or was just inserted");

                    pubber
                        .async_publish(&pointcloud2_msg(cloud_msg))
                        .await
                        .map_err(|e| anyhow!("{e}"))?;
                }
                mt_net::SensorTypeMapped::Imu(imu) => {
                    if !imu_publishers.contains_key(&publisher_key) {
                        let mut builder = node.create_pub::<Imu>(&data.topic);
                        if let Some(qos) = qos {
                            builder = builder.with_qos(qos);
                        }
                        imu_publishers.insert(
                            publisher_key.clone(),
                            builder.build().map_err(|e| anyhow!("{e}"))?,
                        );
                    }
                    let pubber = imu_publishers
                        .get(&publisher_key)
                        .expect("Publisher exists or was just inserted");

                    pubber
                        .async_publish(&imu_msg(imu))
                        .await
                        .map_err(|e| anyhow!("{e}"))?;
                }
                mt_net::SensorTypeMapped::Odometry(odometry) => {
                    if !odom_publishers.contains_key(&publisher_key) {
                        let mut builder = node.create_pub::<Odometry>(&data.topic);
                        if let Some(qos) = qos {
                            builder = builder.with_qos(qos);
                        }
                        odom_publishers.insert(
                            publisher_key.clone(),
                            builder.build().map_err(|e| anyhow!("{e}"))?,
                        );
                    }
                    let pubber = odom_publishers
                        .get(&publisher_key)
                        .expect("Publisher exists or was just inserted");

                    pubber
                        .async_publish(&odometry_msg(odometry))
                        .await
                        .map_err(|e| anyhow!("{e}"))?;
                }
                mt_net::SensorTypeMapped::Clock(clock) => {
                    if !clock_publishers.contains_key(&publisher_key) {
                        let mut builder = node.create_pub::<Clock>(&data.topic);
                        if let Some(qos) = qos {
                            builder = builder.with_qos(qos);
                        }
                        clock_publishers.insert(
                            publisher_key.clone(),
                            builder.build().map_err(|e| anyhow!("{e}"))?,
                        );
                    }
                    let pubber = clock_publishers
                        .get(&publisher_key)
                        .expect("Publisher exists or was just inserted");

                    pubber
                        .async_publish(&clock_msg(clock))
                        .await
                        .map_err(|e| anyhow!("{e}"))?;
                }
                mt_net::SensorTypeMapped::Any(raw_data) => {
                    let any_publisher_key = (
                        data.topic.clone(),
                        data.msg_type.clone(),
                        data.type_description_hash.clone(),
                    );
                    if !any_publishers.contains_key(&any_publisher_key) {
                        let type_hash = type_hash_or_zero(
                            &data.topic,
                            &data.msg_type,
                            data.type_description_hash.as_deref(),
                            &mut malformed_hash_warned,
                        );
                        let type_info = TypeInfo::new(&data.msg_type, type_hash);
                        let mut builder = node
                            .create_pub_impl::<DynamicMessage>(&data.topic, Some(type_info))
                            .with_serdes::<DynamicSerdeCdrSerdes>();
                        if let Some(qos) = qos {
                            builder = builder.with_qos(qos);
                        }
                        any_publishers.insert(
                            any_publisher_key.clone(),
                            builder.build().map_err(|e| anyhow!("{e}"))?,
                        );
                    }
                    let pubber = any_publishers
                        .get(&any_publisher_key)
                        .expect("Publisher exists or was just inserted");

                    pubber
                        .publish_serialized(raw_data)
                        .map_err(|e| anyhow!("{e}"))?;
                }
            }
        }
    }

    Ok(())
}

fn time_msg(time: ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time) -> Time {
    Time {
        sec: time.sec,
        nanosec: time.nanosec,
    }
}

fn header_msg(header: ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header) -> Header {
    Header {
        stamp: time_msg(header.stamp),
        frame_id: header.frame_id,
    }
}

fn pointcloud2_msg(msg: ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2) -> PointCloud2 {
    PointCloud2 {
        header: header_msg(msg.header),
        height: msg.height,
        width: msg.width,
        fields: msg
            .fields
            .into_iter()
            .map(|field| PointField {
                name: field.name,
                offset: field.offset,
                datatype: field.datatype,
                count: field.count,
            })
            .collect(),
        is_bigendian: msg.is_bigendian,
        point_step: msg.point_step,
        row_step: msg.row_step,
        data: msg.data.into(),
        is_dense: msg.is_dense,
    }
}

fn quaternion_msg(msg: ros2_interfaces_jazzy_rkyv::geometry_msgs::msg::Quaternion) -> Quaternion {
    Quaternion {
        x: msg.x,
        y: msg.y,
        z: msg.z,
        w: msg.w,
    }
}

fn vector3_msg(msg: ros2_interfaces_jazzy_rkyv::geometry_msgs::msg::Vector3) -> Vector3 {
    Vector3 {
        x: msg.x,
        y: msg.y,
        z: msg.z,
    }
}

fn imu_msg(msg: ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::Imu) -> Imu {
    Imu {
        header: header_msg(msg.header),
        orientation: quaternion_msg(msg.orientation),
        orientation_covariance: msg.orientation_covariance,
        angular_velocity: vector3_msg(msg.angular_velocity),
        angular_velocity_covariance: msg.angular_velocity_covariance,
        linear_acceleration: vector3_msg(msg.linear_acceleration),
        linear_acceleration_covariance: msg.linear_acceleration_covariance,
    }
}

fn point_msg(msg: ros2_interfaces_jazzy_rkyv::geometry_msgs::msg::Point) -> Point {
    Point {
        x: msg.x,
        y: msg.y,
        z: msg.z,
    }
}

fn odometry_msg(msg: ros2_interfaces_jazzy_rkyv::nav_msgs::msg::Odometry) -> Odometry {
    Odometry {
        header: header_msg(msg.header),
        child_frame_id: msg.child_frame_id,
        pose: PoseWithCovariance {
            pose: Pose {
                position: point_msg(msg.pose.pose.position),
                orientation: quaternion_msg(msg.pose.pose.orientation),
            },
            covariance: msg.pose.covariance,
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: vector3_msg(msg.twist.twist.linear),
                angular: vector3_msg(msg.twist.twist.angular),
            },
            covariance: msg.twist.covariance,
        },
    }
}

fn clock_msg(msg: ros2_interfaces_jazzy_rkyv::rosgraph_msgs::msg::Clock) -> Clock {
    Clock {
        clock: time_msg(msg.clock),
    }
}

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
    mt_log::init_filtered("Wind", "WIND_LOG", "info", mt_log::QUIET_ZENOH);

    let wind_name = get_env_or_default("wind_hiroz_name", "turbine_hiroz")?;

    let (tx, rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        _ = rx.await;
    });
    run_dyn_wind(&wind_name, tx).await?;

    Ok(())
}
