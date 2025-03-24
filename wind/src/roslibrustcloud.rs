roslibrust::find_and_generate_ros_messages_without_ros_package_path!("msg/ros1");

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
