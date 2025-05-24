roslibrust::find_and_generate_ros_messages_without_ros_package_path!("msg/ros1");

use anyhow::anyhow;
use std::collections::HashMap;

use log::{debug, error, info};
use roslibrust::{
    codegen::Time,
    ros1::{NodeHandle, RosMasterError},
};
use sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

use ros_pointcloud2::{CloudDimensions, Denseness, Endian, prelude::*};
use roslibrust::codegen::integral_types::Time as RosTime;

impl From<sensor_msgs::PointCloud2> for PointCloud2Msg {
    fn from(msg: sensor_msgs::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: msg.header.seq,
                stamp: TimeMsg {
                    sec: msg.header.stamp.secs as i32,
                    nanosec: msg.header.stamp.nsecs as u32,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                Endian::Big
            } else {
                Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                Denseness::Dense
            } else {
                Denseness::Sparse
            },
        }
    }
}

impl From<PointCloud2Msg> for sensor_msgs::PointCloud2 {
    fn from(msg: PointCloud2Msg) -> Self {
        sensor_msgs::PointCloud2 {
            header: std_msgs::Header {
                seq: msg.header.seq,
                stamp: RosTime {
                    secs: msg.header.stamp.sec,
                    nsecs: msg.header.stamp.nanosec as i32,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(|field| sensor_msgs::PointField {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            is_bigendian: if msg.endian == Endian::Big {
                true
            } else {
                false
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: if msg.dense == Denseness::Dense {
                true
            } else {
                false
            },
        }
    }
}

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

pub async fn run_dyn_wind(master_uri: &str, wind_name: &str) -> anyhow::Result<Option<()>> {
    let nh = NodeHandle::new(&master_uri, &wind_name).await;

    let nh = match nh {
        Ok(nh) => nh,
        Err(err) => match err {
            roslibrust::ros1::NodeError::RosMasterError(
                RosMasterError::ServerCommunicationFailure(_),
            )
            | roslibrust::ros1::NodeError::RosMasterError(
                RosMasterError::HostIpResolutionFailure(_),
            ) => return Ok(None), // reports that there was no connection to the ROS1 master, print as error outside and stop here
            _ => {
                return Err(err.into());
            }
        },
    };
    let mut cloud_publishers = HashMap::new();
    let mut imu_publishers = HashMap::new();

    let mut wind_receiver = wind(&wind_name).await?;

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                sea::SensorTypeMapped::Lidar(cloud_msg) => {
                    let mut existing_pubber = cloud_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = nh
                            .advertise::<sensor_msgs::PointCloud2>(&data.topic, 10, false)
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
                    let msg = sensor_msgs::PointCloud2 {
                        header: std_msgs::Header {
                            seq: 0,
                            stamp: Time {
                                secs: cloud_msg.header.stamp.sec,
                                nsecs: i32::try_from(cloud_msg.header.stamp.nanosec)?,
                            },
                            frame_id: cloud_msg.header.frame_id,
                        },
                        height: cloud_msg.height,
                        width: cloud_msg.width,
                        fields: cloud_msg
                            .fields
                            .into_iter()
                            .map(|pf| sensor_msgs::PointField {
                                name: pf.name,
                                offset: pf.offset,
                                datatype: pf.datatype,
                                count: pf.count,
                            })
                            .collect::<Vec<_>>(),
                        is_bigendian: cloud_msg.is_bigendian,
                        point_step: cloud_msg.point_step,
                        row_step: cloud_msg.row_step,
                        data: cloud_msg.data,
                        is_dense: cloud_msg.is_dense,
                    };
                    pubber.publish(&msg).await?;
                    debug!("published cloud");
                }
                sea::SensorTypeMapped::Imu(imu_msg) => {
                    let mut existing_pubber = imu_publishers.get(&data.topic);
                    if existing_pubber.is_none() {
                        let pubber = nh
                            .advertise::<sensor_msgs::Imu>(&data.topic, 10, false)
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
                    let msg = sensor_msgs::Imu {
                        header: std_msgs::Header {
                            seq: 0,
                            stamp: Time {
                                secs: imu_msg.header.stamp.sec,
                                nsecs: i32::try_from(imu_msg.header.stamp.nanosec)?,
                            },
                            frame_id: imu_msg.header.frame_id,
                        },
                        orientation: {
                            geometry_msgs::Quaternion {
                                x: imu_msg.orientation.x,
                                y: imu_msg.orientation.y,
                                z: imu_msg.orientation.z,
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
                        angular_velocity: geometry_msgs::Vector3 {
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
                        linear_acceleration: geometry_msgs::Vector3 {
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
                    pubber.publish(&msg).await?;
                    debug!("published cloud");
                }
                sea::SensorTypeMapped::Any(_) => {
                    error!(
                        "Any-Types are not supported for ROS1 due to different message encodings."
                    )
                }
            }
        }
    }

    Ok(Some(()))
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
    env_logger::init();

    let wind_name = get_env_or_default("wind_name", "turbine_ros1")?;
    let master_uri = get_env_or_default("ROS_MASTER_URI", "http://localhost:11311")?;

    run_dyn_wind(&master_uri, &wind_name).await?;

    Ok(())
}
