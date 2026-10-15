use super::*;
use std::collections::HashMap;

fn topic_sensor(topic: &str) -> Vec<AnySensor> {
    vec![AnySensor {
        name: topic.into(),
        id: SensorIdentification::Topic(topic.into()),
        short: None,
    }]
}

/// Writes `(topic, log_time)` records; payloads are opaque because topic
/// queries pass messages through undecoded.
fn bag(records: &[(&str, u64)]) -> Bagfile {
    let mut io = std::io::Cursor::new(Vec::new());
    {
        let mut writer = mcap::Writer::new(&mut io).unwrap();
        let mut channels = HashMap::new();
        for (topic, ros_type) in [
            ("/imu", "sensor_msgs/msg/Imu"),
            ("/cloud", "sensor_msgs/msg/PointCloud2"),
        ] {
            let schema = writer.add_schema(ros_type, "ros2msg", b"").unwrap();
            let channel = writer
                .add_channel(schema, topic, "cdr", &Default::default())
                .unwrap();
            channels.insert(topic, channel);
        }
        for (topic, log_time) in records {
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader {
                        channel_id: channels[topic],
                        sequence: 0,
                        log_time: *log_time,
                        publish_time: *log_time,
                    },
                    &[1, 2, 3],
                )
                .unwrap();
        }
        writer.finish().unwrap();
    }
    let mut bag = Bagfile::default();
    bag.reset_with_io(Box::new(std::io::Cursor::new(io.into_inner())), None)
        .unwrap();
    bag
}

/// The IMUs up to the next cloud, as `pf! i, l 1` asks for them.
fn imus_until_cloud(bag: &mut Bagfile) -> BagReadResult {
    collect_until(
        bag.reader.as_mut().unwrap(),
        bag.io.as_mut().unwrap(),
        &mut bag.read_buffer,
        &mut bag.last_iter_time,
        &mut bag.start_time,
        &bag.summary,
        bag.metadata.as_ref().unwrap(),
        &topic_sensor("/imu"),
        Some((&topic_sensor("/cloud"), &PlayCount::Amount(1))),
        None,
    )
    .unwrap()
}

/// One cloud, as `pf! l 1` asks for it.
fn one_cloud(bag: &mut Bagfile) -> BagReadResult {
    collect_until(
        bag.reader.as_mut().unwrap(),
        bag.io.as_mut().unwrap(),
        &mut bag.read_buffer,
        &mut bag.last_iter_time,
        &mut bag.start_time,
        &bag.summary,
        bag.metadata.as_ref().unwrap(),
        &topic_sensor("/cloud"),
        None,
        Some(&PlayCount::Amount(1)),
    )
    .unwrap()
}

fn times(result: &BagReadResult) -> Vec<u64> {
    result.messages.iter().map(|(t, _)| *t).collect()
}

#[test]
fn exhausted_send_stream_ends_the_bag_while_other_topics_remain() {
    let mut bag = bag(&[
        ("/imu", 1),
        ("/cloud", 2),
        ("/imu", 3),
        ("/cloud", 4),
        ("/cloud", 5),
        ("/cloud", 6),
    ]);
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![1], false));
    assert_eq!(times(&one_cloud(&mut bag)), vec![2]);
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![3], false));
    assert_eq!(times(&one_cloud(&mut bag)), vec![4]);
    // Clouds 5 and 6 remain, but no IMU does: without the end, the replay waits forever.
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![], true));
}

#[test]
fn gap_in_send_stream_is_not_the_end() {
    let mut bag = bag(&[
        ("/imu", 1),
        ("/cloud", 2),
        ("/cloud", 3),
        ("/imu", 4),
        ("/cloud", 5),
    ]);
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![1], false));
    assert_eq!(times(&one_cloud(&mut bag)), vec![2]);
    // No IMU between clouds 2 and 3, but one follows later.
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![], false));
    assert_eq!(times(&one_cloud(&mut bag)), vec![3]);
    let r = imus_until_cloud(&mut bag);
    assert_eq!((times(&r), r.end_of_bag), (vec![4], false));
}
