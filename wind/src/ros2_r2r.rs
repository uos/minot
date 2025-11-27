use std::collections::HashMap;
use std::str::FromStr;
use std::sync::Mutex;

use bagread::qos::{
    RmwQosDurabilityPolicy, RmwQosHistoryPolicy, RmwQosLivelinessPolicy, RmwQosReliabilityPolicy,
};
use r2r::{
    WrappedTypesupport,
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Quaternion, Vector3},
    qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy},
    sensor_msgs::msg::PointField,
    std_msgs::msg::Header,
};

use anyhow::{Context, anyhow};
use log::{debug, error, info, warn};
use sea::{SensorTypeMapped, Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

use r2r::sensor_msgs::msg::{Imu, PointCloud2};

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

#[derive(Copy, Clone, Debug, Default)]
pub enum Qos {
    Sensor,
    #[default]
    SystemDefault,
}

impl TryFrom<String> for Qos {
    type Error = anyhow::Error;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        match value.as_str() {
            "sensor" => Ok(Qos::Sensor),
            "system_default" => Ok(Qos::SystemDefault),
            _ => Err(anyhow!("Invalid QoS value")),
        }
    }
}

impl From<Qos> for r2r::QosProfile {
    fn from(value: Qos) -> Self {
        match value {
            Qos::Sensor => r2r::QosProfile::sensor_data(),
            Qos::SystemDefault => r2r::QosProfile::system_default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct QosR2RMap(r2r::QosProfile);

impl TryFrom<bagread::Qos> for QosR2RMap {
    type Error = anyhow::Error;
    fn try_from(value: bagread::Qos) -> Result<Self, Self::Error> {
        Ok(match value {
            bagread::Qos::Sensor => Self(r2r::QosProfile::sensor_data()),
            bagread::Qos::SystemDefault => Self(r2r::QosProfile::system_default()),
            bagread::Qos::Custom(q) => {
                let deadline = std::time::Duration::from_secs(q.deadline.sec)
                    + std::time::Duration::from_nanos(q.deadline.nsec);
                let lifespan = std::time::Duration::from_secs(q.lifespan.sec)
                    + std::time::Duration::from_nanos(q.lifespan.nsec);

                let reliability = match RmwQosReliabilityPolicy::from_str(&q.reliability)? {
                    RmwQosReliabilityPolicy::SystemDefault => ReliabilityPolicy::SystemDefault,
                    RmwQosReliabilityPolicy::Reliable => ReliabilityPolicy::Reliable,
                    RmwQosReliabilityPolicy::BestEffort => ReliabilityPolicy::BestEffort,
                    RmwQosReliabilityPolicy::Unknown => ReliabilityPolicy::Unknown,
                    #[cfg(not(feature = "humble"))]
                    RmwQosReliabilityPolicy::BestAvailable => ReliabilityPolicy::BestAvailable,
                };

                let durability = match RmwQosDurabilityPolicy::from_str(&q.durability)? {
                    RmwQosDurabilityPolicy::SystemDefault => DurabilityPolicy::SystemDefault,
                    RmwQosDurabilityPolicy::TransientLocal => DurabilityPolicy::TransientLocal,
                    RmwQosDurabilityPolicy::Volatile => DurabilityPolicy::Volatile,
                    RmwQosDurabilityPolicy::Unknown => DurabilityPolicy::Unknown,
                    #[cfg(not(feature = "humble"))]
                    RmwQosDurabilityPolicy::BestAvailable => DurabilityPolicy::BestAvailable,
                };

                let history = match RmwQosHistoryPolicy::from_str(&q.history)? {
                    RmwQosHistoryPolicy::SystemDefault => HistoryPolicy::SystemDefault,
                    RmwQosHistoryPolicy::KeepLast => HistoryPolicy::KeepLast,
                    RmwQosHistoryPolicy::KeepAll => HistoryPolicy::KeepAll,
                    RmwQosHistoryPolicy::Unknown => HistoryPolicy::Unknown,
                };

                let depth = q.depth.max(0) as usize;

                let liveliness_dur =
                    std::time::Duration::from_secs(q.liveliness_lease_duration.sec)
                        + std::time::Duration::from_nanos(q.liveliness_lease_duration.nsec);
                let liveliness = match RmwQosLivelinessPolicy::from_str(&q.liveliness)? {
                    RmwQosLivelinessPolicy::SystemDefault => LivelinessPolicy::SystemDefault,
                    RmwQosLivelinessPolicy::Automatic => LivelinessPolicy::Automatic,
                    RmwQosLivelinessPolicy::ManualByNode => LivelinessPolicy::ManualByNode,
                    RmwQosLivelinessPolicy::ManualByTopic => LivelinessPolicy::ManualByTopic,
                    RmwQosLivelinessPolicy::Unknown => LivelinessPolicy::Unknown,
                    #[cfg(not(feature = "humble"))]
                    RmwQosLivelinessPolicy::BestAvailable => LivelinessPolicy::BestAvailable,
                };

                // let lease_dur = q.liveliness_lease_duration
                Self(r2r::QosProfile {
                    history,
                    depth,
                    reliability,
                    durability,
                    deadline,
                    lifespan,
                    liveliness,
                    liveliness_lease_duration: liveliness_dur,
                    avoid_ros_namespace_conventions: false,
                })
            }
        })
    }
}
pub async fn run_dyn_wind(
    wind_name: &str,
    ready: tokio::sync::oneshot::Sender<()>,
) -> anyhow::Result<()> {
    let node_name = wind_name.to_owned() + "_node";
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, &node_name, "")?;
    let node = std::sync::Arc::new(Mutex::new(node));

    let mut publishers: HashMap<String, r2r::PublisherUntyped> = HashMap::new();

    let mut wind_receiver = wind(wind_name).await?;
    
    if let Err(_) = ready.send(()) {
        log::warn!("ros2_r2r wind could not signal to be ready to handle requests");
    }

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            let qos = match data.qos {
                Some(qos) => QosR2RMap::try_from(qos)?.0,
                None => {
                    warn!("Received message without QOS, choosing system default");
                    // TODO debounce warning or only use info
                    Qos::default().into()
                }
            };
            let mut existing_pubber = publishers.get(&data.topic);
            if existing_pubber.is_none() {
                let mut lock = node.lock().unwrap();
                let pubber = lock.create_publisher_untyped(&data.topic, &data.msg_type, qos)?;

                publishers.insert(data.topic.clone(), pubber);
                existing_pubber = Some(
                    publishers
                        .get(&data.topic)
                        .expect("Just inserted the line before"),
                );
            }
            let pubber = existing_pubber.expect("Should be inserted manually if not exists.");
            match data.data {
                SensorTypeMapped::Lidar(l) => {
                    let native_t = PointCloud2 {
                        header: Header {
                            stamp: Time {
                                sec: l.header.stamp.sec,
                                nanosec: l.header.stamp.nanosec,
                            },
                            frame_id: l.header.frame_id,
                        },
                        height: l.height,
                        width: l.width,
                        fields: l
                            .fields
                            .into_iter()
                            .map(|f| PointField {
                                name: f.name,
                                offset: f.offset,
                                datatype: f.datatype,
                                count: f.count,
                            })
                            .collect(),
                        is_bigendian: l.is_bigendian,
                        point_step: l.point_step,
                        row_step: l.row_step,
                        data: l.data,
                        is_dense: l.is_dense,
                    };
                    let raw = native_t
                        .to_serialized_bytes()
                        .context("Error encoding CDR")?;

                    pubber.publish_raw(&raw)?;

                    debug!("published cloud");
                }
                SensorTypeMapped::Imu(imu) => {
                    let native_t = Imu {
                        header: Header {
                            stamp: Time {
                                sec: imu.header.stamp.sec,
                                nanosec: imu.header.stamp.nanosec,
                            },
                            frame_id: imu.header.frame_id,
                        },
                        orientation: Quaternion {
                            x: imu.orientation.x,
                            y: imu.orientation.y,
                            z: imu.orientation.z,
                            w: imu.orientation.w,
                        },
                        orientation_covariance: imu.orientation_covariance.to_vec(),
                        angular_velocity: Vector3 {
                            x: imu.angular_velocity.x,
                            y: imu.angular_velocity.y,
                            z: imu.angular_velocity.z,
                        },
                        angular_velocity_covariance: imu.angular_velocity_covariance.to_vec(),
                        linear_acceleration: Vector3 {
                            x: imu.linear_acceleration.x,
                            y: imu.linear_acceleration.y,
                            z: imu.linear_acceleration.z,
                        },
                        linear_acceleration_covariance: imu.linear_acceleration_covariance.to_vec(),
                    };
                    let raw = native_t
                        .to_serialized_bytes()
                        .context("Error encoding CDR: {e}")?;

                    pubber.publish_raw(&raw)?;
                    debug!("published imu");
                }
                SensorTypeMapped::Any(raw_data) => {
                    pubber.publish_raw(&raw_data)?;
                    debug!("published raw");
                }
            }
        }
    }

    Ok(())
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
    let env = env_logger::Env::new().filter_or("WIND_LOG", "info");
    env_logger::Builder::from_env(env).init();

    let wind_name = get_env_or_default("WIND_NAME", "turbine_ros2_c")?;

    let (tx, rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        // dump ready answer
        _ = rx.await;
    });
    run_dyn_wind(&wind_name, tx).await?; // will never return

    Ok(())
}
