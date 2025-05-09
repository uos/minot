use std::path::PathBuf;
use std::str::FromStr;

use ros2_client::{NodeName, NodeOptions};
use ros2_client::{Publisher, ros2};
use ros2_interfaces_jazzy::sensor_msgs::msg::Imu;
use ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2;

use anyhow::anyhow;
use log::{error, info};
use sea::SensorTypeMapped;
use wind::wind;

fn get_env_or_default(key: &str, default: &str) -> anyhow::Result<String> {
    match std::env::var(key) {
        Ok(name) => Ok(name),
        Err(e) => match e {
            std::env::VarError::NotPresent => Ok(default.to_owned()),
            std::env::VarError::NotUnicode(_os_string) => {
                error!("Could not fetch node name because it is not unicode");
                Err(anyhow!("Could not initialize node"))
            }
        },
    }
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

impl From<crate::Qos> for ros2::QosPolicies {
    fn from(value: crate::Qos) -> Self {
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

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let env = env_logger::Env::new().filter_or("WIND_LOG", "off");
    env_logger::Builder::from_env(env).init();

    let wind_name = get_env_or_default("wind_name", "turbine_ros2")?;
    let rlc_file = if let Some(file) = std::env::args().nth(1) {
        file
    } else {
        get_env_or_default("CONFIG_PATH", "./wind_ros2.rl")?
    };

    let eval = rlc::compile_file(&PathBuf::from_str(&rlc_file)?, None, None)?;

    let rlc_wind_ns = eval.vars.filter_ns(&[&wind_name]);

    let imu_qos = Qos::try_from(rlc_wind_ns.resolve("_imu.qos")?.map_or(
        Ok("sensor".to_owned()),
        |rhs| {
            Ok(match rhs {
                rlc::Rhs::Val(rlc::Val::StringVal(topic)) => topic,
                _ => {
                    return Err(anyhow!("Unexpected type for _imu.qos"));
                }
            })
        },
    )?)?;

    let cloud_qos = Qos::try_from(rlc_wind_ns.resolve("_lidar.qos")?.map_or(
        Ok("sensor".to_owned()),
        |rhs| {
            Ok(match rhs {
                rlc::Rhs::Val(rlc::Val::StringVal(topic)) => topic,
                _ => {
                    return Err(anyhow!("Unexpected type for _lidar.qos, expected String."));
                }
            })
        },
    )?)?;

    let namespace = rlc_wind_ns
        .resolve("namespace")?
        .map_or(Ok("/wind".to_owned()), |rhs| {
            Ok(match rhs {
                rlc::Rhs::Path(topic) => topic,
                _ => {
                    return Err(anyhow!("Unexpected type for _lidar.topic, expected Path."));
                }
            })
        })?;

    let ctx = ros2_client::Context::new()?;
    let node_name = wind_name.clone() + "_node";
    let mut node = ctx.new_node(
        NodeName::new(&namespace, &node_name)?,
        NodeOptions::new().enable_rosout(true),
    )?;

    // let mut publisher = HashMap::new();

    let mut cloud_publisher: Option<Publisher<PointCloud2>> = None;
    let mut current_cloud_topic = ros2_client::Name::new("/wind", "wind_cloud").unwrap();

    let mut imu_publisher: Option<Publisher<Imu>> = None;
    let mut current_imu_topic = ros2_client::Name::new("/wind", "wind_imu").unwrap();

    let mut wind_receiver = wind(&wind_name).await?;

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            let topic_parse = split_path(&data.topic);
            let wanted_topic = ros2_client::Name::new(&topic_parse.0, &topic_parse.1).unwrap();
            match data.data {
                SensorTypeMapped::Lidar(l) => {
                    let mut trigger_repub = false;
                    if wanted_topic != current_cloud_topic {
                        current_cloud_topic = wanted_topic;
                        trigger_repub = true;
                    }
                    if cloud_publisher.is_none() {
                        trigger_repub = true;
                    }
                    if trigger_repub {
                        cloud_publisher = Some(
                            node.create_publisher(
                                &node
                                    .create_topic(
                                        &current_cloud_topic,
                                        ros2_client::MessageTypeName::new(
                                            "sensor_msgs",
                                            "PointCloud2",
                                        ),
                                        &cloud_qos.into(),
                                    )
                                    .unwrap(),
                                None,
                            )
                            .unwrap(),
                        );
                    }
                    cloud_publisher.as_ref().unwrap().async_publish(l).await?;
                    info!("published cloud");
                }
                SensorTypeMapped::Imu(imu) => {
                    let mut trigger_repub = false;
                    if wanted_topic != current_imu_topic {
                        current_imu_topic = wanted_topic;
                        trigger_repub = true;
                    }
                    if imu_publisher.is_none() {
                        trigger_repub = true;
                    }
                    if trigger_repub {
                        imu_publisher = Some(
                            node.create_publisher(
                                &node
                                    .create_topic(
                                        &current_imu_topic,
                                        ros2_client::MessageTypeName::new("sensor_msgs", "Imu"),
                                        &imu_qos.into(),
                                    )
                                    .unwrap(),
                                None,
                            )
                            .unwrap(),
                        );
                    }
                    imu_publisher.as_ref().unwrap().async_publish(imu).await?;
                }
                SensorTypeMapped::Any(_) => todo!(), // TODO raw publish, then always manually encode the imu and laser messages as well and use hashmap over raw publishers for managing new ones
            }
        }
    }

    Ok(())
}
