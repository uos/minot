use super::*;
use ros2_interfaces_jazzy_serde::{sensor_msgs::msg::*, std_msgs::msg::Header};

fn header() -> Header {
    Header {
        stamp: ros2_interfaces_jazzy_serde::builtin_interfaces::msg::Time {
            sec: 42,
            nanosec: 123_456_789,
        },
        frame_id: "camera_optical_frame".into(),
    }
}

// Exercise bag CDR decoding, the wind wire format, and the native ROS conversion.
macro_rules! check_camera {
    ($msg:expr, $sensor:ident, $ros_type:expr, $convert:ident) => {{
        let msg = $msg;
        for bytes in [
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).unwrap(),
            cdr::serialize::<_, _, cdr::CdrBe>(&msg, cdr::Infinite).unwrap(),
        ] {
            for sensor in [SensorType::$sensor, SensorType::Mixed] {
                let mapped = deserialize_message(&bytes, sensor, $ros_type).unwrap();
                let bag = BagMsg {
                    topic: "/camera/test".into(),
                    msg_type: $ros_type.into(),
                    type_description_hash: None,
                    data: mapped,
                    qos: Some(Qos::Sensor),
                };
                let wire = rkyv::to_bytes::<rkyv::rancor::Error>(&bag).unwrap();
                let decoded = rkyv::from_bytes::<BagMsg, rkyv::rancor::Error>(&wire).unwrap();
                assert_eq!(decoded.topic, bag.topic);
                assert_eq!(decoded.msg_type, bag.msg_type);
                match decoded.data {
                    SensorTypeMapped::$sensor(mapped) => {
                        assert_eq!(camera::$convert(mapped), msg);
                    }
                    other => panic!("Unexpected camera variant: {other:?}"),
                }
            }
        }
        assert!(deserialize_message(&[0, 1, 0, 0], SensorType::$sensor, $ros_type).is_err());
    }};
}

#[test]
fn image_bag_and_wire_roundtrip() {
    check_camera!(
        Image {
            header: header(),
            height: 2,
            width: 2,
            encoding: "mono16".into(),
            is_bigendian: 1,
            step: 6,
            data: vec![0, 1, 2, 3, 0, 0, 128, 255, 4, 5, 0, 0],
        },
        Image,
        mt_mtc::IMAGE_ROS2_TYPE,
        image_to_serde
    );
}

#[test]
fn compressed_image_bag_and_wire_roundtrip() {
    check_camera!(
        CompressedImage {
            header: header(),
            format: "rgb8; jpeg compressed bgr8".into(),
            data: vec![255, 216, 0, 128, 255, 217],
        },
        CompressedImage,
        mt_mtc::COMPRESSED_IMAGE_ROS2_TYPE,
        compressed_image_to_serde
    );
}

#[test]
fn camera_info_bag_and_wire_roundtrip() {
    check_camera!(
        CameraInfo {
            header: header(),
            height: 480,
            width: 640,
            distortion_model: "plumb_bob".into(),
            d: vec![0.1, -0.2, 0.3, 0.4, 0.5],
            k: [501.0, 1.0, 320.0, 0.0, 502.0, 240.0, 0.0, 0.0, 1.0],
            r: [0.9, 0.1, 0.2, 0.3, 0.8, 0.4, 0.5, 0.6, 0.7],
            p: [
                501.0, 0.0, 320.0, -50.1, 0.0, 502.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0
            ],
            binning_x: 2,
            binning_y: 3,
            roi: RegionOfInterest {
                x_offset: 10,
                y_offset: 20,
                height: 100,
                width: 200,
                do_rectify: true
            },
        },
        CameraInfo,
        mt_mtc::CAMERA_INFO_ROS2_TYPE,
        camera_info_to_serde
    );
}

#[test]
fn unknown_messages_still_pass_through() {
    let bytes = vec![1, 2, 3];
    assert!(
        matches!(deserialize_message(&bytes, SensorType::Mixed, "custom/msg/Data").unwrap(),
        SensorTypeMapped::Any(data) if data == bytes)
    );
    assert!(
        matches!(deserialize_message(&bytes, SensorType::Any, mt_mtc::IMAGE_ROS2_TYPE).unwrap(),
        SensorTypeMapped::Any(data) if data == bytes)
    );
}

fn camera_sensors() -> Vec<AnySensor> {
    [
        SensorType::Image,
        SensorType::CompressedImage,
        SensorType::CameraInfo,
    ]
    .into_iter()
    .map(|sensor| AnySensor {
        name: format!("{sensor:?}"),
        id: SensorIdentification::Type(sensor),
        short: None,
        passive: false,
    })
    .collect()
}

fn camera_records() -> Vec<(&'static str, Vec<u8>)> {
    vec![
        (
            mt_mtc::IMAGE_ROS2_TYPE,
            cdr::serialize::<_, _, cdr::CdrLe>(
                &Image {
                    header: header(),
                    data: vec![1, 2, 3],
                    ..Default::default()
                },
                cdr::Infinite,
            )
            .unwrap(),
        ),
        (
            mt_mtc::COMPRESSED_IMAGE_ROS2_TYPE,
            cdr::serialize::<_, _, cdr::CdrLe>(
                &CompressedImage {
                    header: header(),
                    format: "jpeg".into(),
                    data: vec![255, 216],
                },
                cdr::Infinite,
            )
            .unwrap(),
        ),
        (
            mt_mtc::CAMERA_INFO_ROS2_TYPE,
            cdr::serialize::<_, _, cdr::CdrLe>(
                &CameraInfo {
                    header: header(),
                    width: 640,
                    ..Default::default()
                },
                cdr::Infinite,
            )
            .unwrap(),
        ),
    ]
}

fn assert_camera_records(messages: &[(u64, BagMsg)]) {
    assert_eq!(messages.len(), 3);
    assert!(
        matches!(&messages[0].1.data, SensorTypeMapped::Image(msg) if msg.data == [1, 2, 3] && msg.header.frame_id == "camera_optical_frame")
    );
    assert!(
        matches!(&messages[1].1.data, SensorTypeMapped::CompressedImage(msg) if msg.data == [255, 216] && msg.format == "jpeg")
    );
    assert!(matches!(&messages[2].1.data, SensorTypeMapped::CameraInfo(msg) if msg.width == 640));
}

#[test]
fn camera_type_queries_read_mcap_and_stream() {
    let mut io = std::io::Cursor::new(Vec::new());
    {
        let mut writer = mcap::Writer::new(&mut io).unwrap();
        for (index, (ros_type, bytes)) in camera_records().into_iter().enumerate() {
            let schema = writer.add_schema(ros_type, "ros2msg", b"").unwrap();
            let channel = writer
                .add_channel(
                    schema,
                    &format!("/camera/{index}"),
                    "cdr",
                    &Default::default(),
                )
                .unwrap();
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader {
                        channel_id: channel,
                        sequence: 0,
                        log_time: index as u64 + 1,
                        publish_time: index as u64 + 1,
                    },
                    &bytes,
                )
                .unwrap();
        }
        writer.finish().unwrap();
    }
    let bytes = io.into_inner();
    let mut bag = Bagfile::default();
    bag.reset_with_io(Box::new(std::io::Cursor::new(bytes.clone())), None)
        .unwrap();
    let result = collect_until(
        bag.reader.as_mut().unwrap(),
        bag.io.as_mut().unwrap(),
        &mut bag.read_buffer,
        &mut bag.last_iter_time,
        &mut bag.start_time,
        &bag.summary,
        bag.metadata.as_ref().unwrap(),
        &camera_sensors(),
        None,
        None,
    )
    .unwrap();
    assert_camera_records(&result.messages);
    assert!(result.end_of_bag);
    bag.reset_with_io(Box::new(std::io::Cursor::new(bytes)), None)
        .unwrap();
    let mut streamed = Vec::new();
    while let Some(msg) = bag.next_message().unwrap() {
        streamed.push(msg);
    }
    assert_camera_records(&streamed);
}

#[cfg(feature = "db3")]
#[test]
fn camera_type_queries_read_db3() {
    let conn = Connection::open_in_memory().unwrap();
    conn.execute(
        "CREATE TABLE messages (timestamp INTEGER, topic_id INTEGER, data BLOB)",
        [],
    )
    .unwrap();
    let mut topic_map = std::collections::HashMap::new();
    for (index, (ros_type, bytes)) in camera_records().into_iter().enumerate() {
        let id = index as i64 + 1;
        topic_map.insert(id, (format!("/camera/{index}"), ros_type.into(), vec![]));
        conn.execute(
            "INSERT INTO messages VALUES (?1, ?2, ?3)",
            rusqlite::params![id, id, bytes],
        )
        .unwrap();
    }
    let mut db3 = Db3State {
        conn: std::sync::Mutex::new(conn),
        cursor_ns: None,
        start_ns: None,
        topic_map,
    };
    let result = collect_until_db3(&mut db3, &camera_sensors(), None, None).unwrap();
    assert_camera_records(&result.messages);
    assert!(result.end_of_bag);
}

#[test]
fn camera_stop_sensor_matches_message_type() {
    let mut count = 0;
    let sensors = camera_sensors();
    assert!(
        !check_until_sensor(
            &sensors,
            "/camera/image",
            IMU_ROS2_TYPE,
            &mut count,
            &PlayCount::Amount(1)
        )
        .unwrap()
    );
    assert_eq!(count, 0);
    assert!(
        check_until_sensor(
            &sensors,
            "/camera/image",
            mt_mtc::IMAGE_ROS2_TYPE,
            &mut count,
            &PlayCount::Amount(1)
        )
        .unwrap()
    );
    assert_eq!(count, 1);
}
