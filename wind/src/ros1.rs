mod roslibrustcloud;

use anyhow::anyhow;
use log::error;
use roslibrust::{codegen::Time, ros1::NodeHandle};
use roslibrustcloud::{sensor_msgs::PointField, *};
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

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let wind_name = get_env_or_default("wind_name", "turbine_ros1")?;
    let node_uri = get_env_or_default("NODE_URI", "http://localhost:11311")?;
    let cloud_topic = get_env_or_default("CLOUD_TOPIC", "/wind_cloud")?;
    let imu_topic = get_env_or_default("IMU_TOPIC", "/wind_imu")?;

    let nh = NodeHandle::new(&node_uri, &wind_name)
        .await
        .map_err(|err| err)?;

    let cloud_publisher = nh
        .advertise::<sensor_msgs::PointCloud2>(&cloud_topic, 10, false)
        .await?;

    let imu_publisher = nh
        .advertise::<sensor_msgs::Imu>(&imu_topic, 10, false)
        .await?;

    let mut wind_receiver = wind(&wind_name).await?;

    while let Some(wind_data) = wind_receiver.recv().await {
        for data in wind_data {
            match data.data {
                sea::SensorTypeMapped::Lidar(cloud_msg) => {
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
                            .map(|pf| PointField {
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
                    cloud_publisher.publish(&msg).await?;
                }
                sea::SensorTypeMapped::Imu(imu_msg) => {
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
                    imu_publisher.publish(&msg).await?;
                }
                sea::SensorTypeMapped::Any(_) => {
                    error!("Any-Types are not supported for ROS1.")
                }
            }
        }
    }

    Ok(())
}
