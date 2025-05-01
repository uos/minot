use std::path::PathBuf;
use std::str::FromStr;

use ros2_client::ros2;
use ros2_client::{NodeName, NodeOptions};
use ros2_interfaces_jazzy::geometry_msgs::msg::Vector3;
use ros2_interfaces_jazzy::sensor_msgs::msg::Imu;
use ros2_interfaces_jazzy::{
    builtin_interfaces::msg::Time, geometry_msgs::msg::Quaternion, sensor_msgs::msg::PointCloud2,
    std_msgs::msg::Header,
};

use anyhow::anyhow;
use log::{error, info};
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

    let imu_topic =
        rlc_wind_ns
            .resolve("_imu.topic")?
            .map_or(Ok("wind_imu".to_owned()), |rhs| {
                Ok(match rhs {
                    rlc::Rhs::Val(rlc::Val::StringVal(topic)) => topic,
                    _ => {
                        return Err(anyhow!("Unexpected type for _imu.topic"));
                    }
                })
            })?;

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

    let cloud_topic =
        rlc_wind_ns
            .resolve("_lidar.topic")?
            .map_or(Ok("wind_lidar".to_owned()), |rhs| {
                Ok(match rhs {
                    rlc::Rhs::Val(rlc::Val::StringVal(topic)) => topic,
                    _ => {
                        return Err(anyhow!(
                            "Unexpected type for _lidar.topic, expected String."
                        ));
                    }
                })
            })?;

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

    let cloud_publisher = node
        .create_publisher(
            &node
                .create_topic(
                    &ros2_client::Name::new(&namespace, &cloud_topic).unwrap(),
                    ros2_client::MessageTypeName::new("sensor_msgs", "PointCloud2"),
                    &cloud_qos.into(),
                )
                .unwrap(),
            None,
        )
        .unwrap();

    let imu_publisher = node
        .create_publisher(
            &node
                .create_topic(
                    &ros2_client::Name::new(&namespace, &imu_topic).unwrap(),
                    ros2_client::MessageTypeName::new("sensor_msgs", "Imu"),
                    &imu_qos.into(),
                )
                .unwrap(),
            None,
        )
        .unwrap();

    let mut wind_receiver = wind(&wind_name).await?;

    while let Some(data) = wind_receiver.recv().await {
        match data {
            wind::sea::WindData::Pointcloud(point_cloud2_msg) => {
                let msg: PointCloud2 = point_cloud2_msg.into();
                cloud_publisher.publish(msg)?;
                info!("published cloud");
            }
            wind::sea::WindData::Imu(imu_msg) => {
                let msg = Imu {
                    header: Header {
                        stamp: Time {
                            sec: imu_msg.header.stamp.sec,
                            nanosec: imu_msg.header.stamp.nanosec,
                        },
                        frame_id: imu_msg.header.frame_id,
                    },
                    orientation: {
                        Quaternion {
                            x: imu_msg.orientation.i,
                            y: imu_msg.orientation.j,
                            z: imu_msg.orientation.k,
                            w: imu_msg.orientation.w,
                        }
                    },
                    orientation_covariance: [
                        imu_msg.orientation_covariance[0],
                        imu_msg.orientation_covariance[1],
                        imu_msg.orientation_covariance[2],
                        imu_msg.orientation_covariance[3],
                        imu_msg.orientation_covariance[4],
                        imu_msg.orientation_covariance[5],
                        imu_msg.orientation_covariance[6],
                        imu_msg.orientation_covariance[7],
                        imu_msg.orientation_covariance[8],
                    ],
                    angular_velocity: Vector3 {
                        x: imu_msg.angular_velocity.x,
                        y: imu_msg.angular_velocity.y,
                        z: imu_msg.angular_velocity.z,
                    },
                    angular_velocity_covariance: [
                        imu_msg.angular_velocity_covariance[0],
                        imu_msg.angular_velocity_covariance[1],
                        imu_msg.angular_velocity_covariance[2],
                        imu_msg.angular_velocity_covariance[3],
                        imu_msg.angular_velocity_covariance[4],
                        imu_msg.angular_velocity_covariance[5],
                        imu_msg.angular_velocity_covariance[6],
                        imu_msg.angular_velocity_covariance[7],
                        imu_msg.angular_velocity_covariance[8],
                    ],
                    linear_acceleration: Vector3 {
                        x: imu_msg.linear_acceleration.x,
                        y: imu_msg.linear_acceleration.y,
                        z: imu_msg.linear_acceleration.z,
                    },
                    linear_acceleration_covariance: [
                        imu_msg.linear_acceleration_covariance[0],
                        imu_msg.linear_acceleration_covariance[1],
                        imu_msg.linear_acceleration_covariance[2],
                        imu_msg.linear_acceleration_covariance[3],
                        imu_msg.linear_acceleration_covariance[4],
                        imu_msg.linear_acceleration_covariance[5],
                        imu_msg.linear_acceleration_covariance[6],
                        imu_msg.linear_acceleration_covariance[7],
                        imu_msg.linear_acceleration_covariance[8],
                    ],
                };
                imu_publisher.publish(msg)?;
                info!("published imu");
            }
        }
    }

    Ok(())
}
