use std::collections::HashMap;

use byteorder::LittleEndian;
use ros2_client::ros2;
use ros2_client::{NodeName, NodeOptions};
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

impl From<Qos> for ros2::QosPolicies {
    fn from(value: Qos) -> Self {
        match value {
            Qos::Sensor => ros2::QosPolicyBuilder::new()
                .durability(ros2::policy::Durability::Volatile)
                .deadline(ros2::policy::Deadline(ros2::Duration::INFINITE))
                .ownership(ros2::policy::Ownership::Shared)
                .reliability(ros2::policy::Reliability::BestEffort)
                .history(ros2::policy::History::KeepLast { depth: 5 })
                .lifespan(ros2::policy::Lifespan {
                    duration: ros2::Duration::INFINITE,
                })
                .build(),
            _ => ros2::QosPolicyBuilder::new()
                .durability(ros2::policy::Durability::Volatile)
                .deadline(ros2::policy::Deadline(ros2::Duration::INFINITE))
                .ownership(ros2::policy::Ownership::Shared)
                .reliability(ros2::policy::Reliability::Reliable {
                    max_blocking_time: ros2::Duration::from_millis(100),
                })
                .history(ros2::policy::History::KeepLast { depth: 1 })
                .lifespan(ros2::policy::Lifespan {
                    duration: ros2::Duration::INFINITE,
                })
                .build(),
        }
    }
}

pub fn split_package_type(path: &str) -> (String, String) {
    let components: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
    let package_name = components.first().unwrap_or(&"").to_string();
    let type_name = components.last().unwrap_or(&"").to_string();

    (package_name, type_name)
}

pub fn split_path(path: &str) -> (String, String) {
    let components: Vec<&str> = path.split('/').collect();
    if components.is_empty() || (components.len() == 1 && components[0].is_empty()) {
        return ("/".to_string(), "".to_string());
    }

    let last_element = components.last().unwrap_or(&"").to_string();
    let dir_components = &components[0..components.len().saturating_sub(1)];
    let directory_path = if path.starts_with('/') {
        dir_components.join("/")
    } else {
        format!("/{}", dir_components.join("/"))
    };

    let final_directory_path = if directory_path.is_empty() {
        "/".to_string()
    } else {
        directory_path
    };

    (final_directory_path, last_element)
}

pub async fn run_dyn_wind(
    wind_name: &str,
    ready: tokio::sync::oneshot::Sender<()>,
) -> anyhow::Result<()> {
    let ctx = ros2_client::Context::new()?;
    let node_name = wind_name.to_owned() + "_node";
    let mut node = ctx.new_node(
        NodeName::new("/", &node_name)?,
        NodeOptions::new().enable_rosout(true),
    )?;
    let mut publishers: HashMap<(String, String), ros2_client::PublisherRaw> = HashMap::new();

    let mut wind_receiver = wind(wind_name).await?;
    
    if let Err(_) = ready.send(()) {
        log::warn!("ros2 wind could not signal to be ready to handle requests");
    }

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            let qos = match data.qos {
                Some(qos) => ros2::QosPolicies::try_from(qos)?,
                None => {
                    warn!("Received message without QOS, choosing system default");
                    // TODO debounce warning or only use info
                    Qos::default().into()
                }
            };
            let topic_parse = split_path(&data.topic);
            let wanted_topic = ros2_client::Name::new(&topic_parse.0, &topic_parse.1).unwrap();
            let (package_name, type_name) = split_package_type(&data.msg_type);
            let pub_type = ros2_client::MessageTypeName::new(&package_name, &type_name);
            let mut existing_pubber = publishers.get(&topic_parse);
            if existing_pubber.is_none() {
                let pubber = node
                    .create_publisher_raw(
                        &node.create_topic(&wanted_topic, pub_type, &qos).unwrap(),
                        None,
                    )
                    .unwrap();
                publishers.insert(topic_parse.clone(), pubber);
                existing_pubber = Some(
                    publishers
                        .get(&topic_parse)
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

                    pubber.async_publish(raw).await?;

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

                    pubber.async_publish(raw).await?;
                    debug!("published imu");
                }
                SensorTypeMapped::Any(raw_data) => {
                    pubber.async_publish(raw_data).await?;
                    debug!("published raw");
                }
            }
        }
    }

    Ok(())
}

// TODO share in module so both submodules can uzse it
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

    let wind_name = get_env_or_default("wind_name", "turbine_ros2")?;

    let (tx, rx) = tokio::sync::oneshot::channel();
    tokio::spawn(async move {
        // dump ready answer
        _ = rx.await;
    });
    run_dyn_wind(&wind_name, tx).await?; // will never return

    Ok(())
}
