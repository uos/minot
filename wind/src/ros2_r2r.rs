use std::collections::HashMap;
use std::str::FromStr;
use std::sync::Mutex;

use bagread::qos::{
    RmwQosDurabilityPolicy, RmwQosHistoryPolicy, RmwQosLivelinessPolicy, RmwQosReliabilityPolicy,
};
use byteorder::LittleEndian;
use r2r::qos::{DurabilityPolicy, HistoryPolicy, LivelinessPolicy, ReliabilityPolicy};
use ros2_interfaces_jazzy::sensor_msgs::msg::Imu;

use anyhow::anyhow;
use log::{debug, error, info, warn};
use sea::{SensorTypeMapped, Ship, ShipKind};
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
                    RmwQosReliabilityPolicy::BestAvailable => ReliabilityPolicy::BestAvailable,
                };

                let durability = match RmwQosDurabilityPolicy::from_str(&q.durability)? {
                    RmwQosDurabilityPolicy::SystemDefault => DurabilityPolicy::SystemDefault,
                    RmwQosDurabilityPolicy::TransientLocal => DurabilityPolicy::TransientLocal,
                    RmwQosDurabilityPolicy::Volatile => DurabilityPolicy::Volatile,
                    RmwQosDurabilityPolicy::Unknown => DurabilityPolicy::Unknown,
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
pub async fn run_dyn_wind(wind_name: &str) -> anyhow::Result<()> {
    let node_name = wind_name.to_owned() + "_node";
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, &node_name, "")?;
    let node = std::sync::Arc::new(Mutex::new(node));

    let mut publishers: HashMap<String, r2r::PublisherUntyped> = HashMap::new();

    let mut wind_receiver = wind(wind_name).await?;

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
                    // reinterpret-cast from rkyv serializable type to serde serial type
                    let l = unsafe {
                        std::mem::transmute::<
                            ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2,
                            _,
                        >(l)
                    };
                    let raw = cdr_encoding::to_vec::<
                        ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2,
                        LittleEndian,
                    >(&l)
                    .map_err(|e| anyhow!("Error encoding CDR: {e}"))?;

                    pubber.publish_raw(&raw)?;

                    debug!("published cloud");
                }
                SensorTypeMapped::Imu(imu) => {
                    let imu = unsafe {
                        std::mem::transmute::<ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::Imu, _>(
                            imu,
                        )
                    };
                    let raw = cdr_encoding::to_vec::<Imu, LittleEndian>(&imu)
                        .map_err(|e| anyhow!("Error encoding CDR: {e}"))?;

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

    run_dyn_wind(&wind_name).await?; // will never return

    Ok(())
}
