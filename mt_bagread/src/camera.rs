//! Field-preserving camera message conversions for bag decoding and ROS publishing.
use ros2_interfaces_jazzy_rkyv as archived;
use ros2_interfaces_jazzy_serde as serial;

pub fn image_to_rkyv(msg: serial::sensor_msgs::msg::Image) -> archived::sensor_msgs::msg::Image {
    archived::sensor_msgs::msg::Image {
        header: archived::std_msgs::msg::Header {
            stamp: archived::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        height: msg.height,
        width: msg.width,
        encoding: msg.encoding,
        is_bigendian: msg.is_bigendian,
        step: msg.step,
        data: msg.data,
    }
}

pub fn compressed_image_to_rkyv(
    msg: serial::sensor_msgs::msg::CompressedImage,
) -> archived::sensor_msgs::msg::CompressedImage {
    archived::sensor_msgs::msg::CompressedImage {
        header: archived::std_msgs::msg::Header {
            stamp: archived::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        format: msg.format,
        data: msg.data,
    }
}

pub fn camera_info_to_rkyv(
    msg: serial::sensor_msgs::msg::CameraInfo,
) -> archived::sensor_msgs::msg::CameraInfo {
    archived::sensor_msgs::msg::CameraInfo {
        header: archived::std_msgs::msg::Header {
            stamp: archived::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        height: msg.height,
        width: msg.width,
        distortion_model: msg.distortion_model,
        d: msg.d,
        k: msg.k,
        r: msg.r,
        p: msg.p,
        binning_x: msg.binning_x,
        binning_y: msg.binning_y,
        roi: archived::sensor_msgs::msg::RegionOfInterest {
            x_offset: msg.roi.x_offset,
            y_offset: msg.roi.y_offset,
            height: msg.roi.height,
            width: msg.roi.width,
            do_rectify: msg.roi.do_rectify,
        },
    }
}

pub fn image_to_serde(msg: archived::sensor_msgs::msg::Image) -> serial::sensor_msgs::msg::Image {
    serial::sensor_msgs::msg::Image {
        header: serial::std_msgs::msg::Header {
            stamp: serial::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        height: msg.height,
        width: msg.width,
        encoding: msg.encoding,
        is_bigendian: msg.is_bigendian,
        step: msg.step,
        data: msg.data,
    }
}

pub fn compressed_image_to_serde(
    msg: archived::sensor_msgs::msg::CompressedImage,
) -> serial::sensor_msgs::msg::CompressedImage {
    serial::sensor_msgs::msg::CompressedImage {
        header: serial::std_msgs::msg::Header {
            stamp: serial::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        format: msg.format,
        data: msg.data,
    }
}

pub fn camera_info_to_serde(
    msg: archived::sensor_msgs::msg::CameraInfo,
) -> serial::sensor_msgs::msg::CameraInfo {
    serial::sensor_msgs::msg::CameraInfo {
        header: serial::std_msgs::msg::Header {
            stamp: serial::builtin_interfaces::msg::Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id: msg.header.frame_id,
        },
        height: msg.height,
        width: msg.width,
        distortion_model: msg.distortion_model,
        d: msg.d,
        k: msg.k,
        r: msg.r,
        p: msg.p,
        binning_x: msg.binning_x,
        binning_y: msg.binning_y,
        roi: serial::sensor_msgs::msg::RegionOfInterest {
            x_offset: msg.roi.x_offset,
            y_offset: msg.roi.y_offset,
            height: msg.roi.height,
            width: msg.roi.width,
            do_rectify: msg.roi.do_rectify,
        },
    }
}
