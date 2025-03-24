mod roslibrustcloud;

use anyhow::anyhow;
use log::error;
use roslibrust::{codegen::Time, ros1::NodeHandle};
use roslibrustcloud::*;
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

    let wind_name = get_env_or_default("wind_name", "turbine-ros1")?;
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

    while let Some(data) = wind_receiver.recv().await {
        match data {
            wind::sea::WindData::Pointcloud(point_cloud2_msg) => {
                let msg: sensor_msgs::PointCloud2 = point_cloud2_msg.into();
                cloud_publisher.publish(&msg).await?;
            }
            wind::sea::WindData::Imu(imu_msg) => {
                let msg = sensor_msgs::Imu {
                    header: std_msgs::Header {
                        seq: imu_msg.header.seq,
                        stamp: Time {
                            secs: imu_msg.header.stamp.sec,
                            nsecs: i32::try_from(imu_msg.header.stamp.nanosec)?,
                        },
                        frame_id: imu_msg.header.frame_id,
                    },
                    orientation: {
                        geometry_msgs::Quaternion {
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
                    angular_velocity: geometry_msgs::Vector3 {
                        x: imu_msg.angular_velocity.x,
                        y: imu_msg.angular_velocity.y,
                        z: imu_msg.angular_velocity.z,
                    },
                    angular_velocity_covariance: [
                        imu_msg.angular_velocity[0],
                        imu_msg.angular_velocity[1],
                        imu_msg.angular_velocity[2],
                        imu_msg.angular_velocity[3],
                        imu_msg.angular_velocity[4],
                        imu_msg.angular_velocity[5],
                        imu_msg.angular_velocity[6],
                        imu_msg.angular_velocity[7],
                        imu_msg.angular_velocity[8],
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
        }
    }

    Ok(())
}
