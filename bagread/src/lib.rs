use std::{fs, path::Path};

use anyhow::{Context, Result, anyhow};
// use camino::Utf8Path;
use mcap::{McapError, Message};
use memmap2::Mmap;
use nalgebra::{UnitQuaternion, Vector3};
use rlc::{PlayCount, PlayKindUnited, PlayTrigger, SensorType};
pub use ros_pointcloud2::PointCloud2Msg;
use ros2_interfaces_jazzy::sensor_msgs::msg::{Imu, PointCloud2};
use serde::{Deserialize, Serialize};
use serde::{Deserializer, de};
use std::fmt;

#[derive(Clone, Debug, Default, Copy, Serialize, Deserialize, PartialEq)]
pub struct TimeMsg {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

#[derive(Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuMsg {
    pub header: Header,
    pub timestamp_sec: TimeMsg,
    pub orientation: UnitQuaternion<f64>,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3<f64>,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3<f64>,
    pub linear_acceleration_covariance: [f64; 9],
}

impl From<Imu> for ImuMsg {
    fn from(value: Imu) -> Self {
        Self {
            header: Header {
                seq: 0,
                stamp: TimeMsg {
                    sec: value.header.stamp.sec,
                    nanosec: value.header.stamp.nanosec,
                },
                frame_id: value.header.frame_id,
            },
            timestamp_sec: TimeMsg {
                sec: value.header.stamp.sec,
                nanosec: value.header.stamp.nanosec,
            },
            orientation: nalgebra::Unit::from_quaternion(nalgebra::Quaternion::from_vector(
                nalgebra::Vector4::new(
                    value.orientation.x,
                    value.orientation.y,
                    value.orientation.z,
                    value.orientation.w,
                ),
            )),
            orientation_covariance: value.orientation_covariance,
            angular_velocity: nalgebra::Vector3::new(
                value.angular_velocity.x,
                value.angular_velocity.y,
                value.angular_velocity.z,
            ),
            angular_velocity_covariance: value.angular_velocity_covariance,
            linear_acceleration: nalgebra::Vector3::new(
                value.linear_acceleration.x,
                value.linear_acceleration.y,
                value.linear_acceleration.z,
            ),
            linear_acceleration_covariance: value.linear_acceleration_covariance,
        }
    }
}

#[derive(Deserialize, Debug)]
pub struct Metadata {
    pub rosbag2_bagfile_information: Rosbag2BagfileInformation,
}

impl Metadata {
    pub fn get_topic_meta(&self, topic: &str) -> Option<&TopicMetadata> {
        self.rosbag2_bagfile_information
            .topics_with_message_count
            .iter()
            .find(|tm| tm.topic_metadata.name == topic)
            .map(|tm| &tm.topic_metadata)
    }
}

// --- Top Level Struct ---

#[derive(Deserialize, Debug)]
pub struct Rosbag2BagfileInformation {
    pub version: i32,
    pub storage_identifier: String,
    pub duration: Nanoseconds,
    pub starting_time: NanosecondsSinceEpoch,
    pub message_count: i64, // Changed to i64 for potentially large counts
    pub topics_with_message_count: Vec<TopicWithMessageCount>,
    pub compression_format: String,
    pub compression_mode: String,
    pub relative_file_paths: Vec<String>,
    pub files: Vec<File>,
}

// --- Time/Duration Structs ---

#[derive(Deserialize, Debug, Clone, Copy)] // Added Clone, Copy
pub struct Nanoseconds {
    pub nanoseconds: u64,
}

#[derive(Deserialize, Debug, Clone, Copy)] // Added Clone, Copy
pub struct NanosecondsSinceEpoch {
    pub nanoseconds_since_epoch: u64,
}

// --- Topic Information Structs ---

#[derive(Deserialize, Debug)]
pub struct TopicWithMessageCount {
    pub topic_metadata: TopicMetadata,
    pub message_count: i64, // Changed to i64
}

#[derive(Deserialize, Debug)]
pub struct TopicMetadata {
    pub name: String,
    #[serde(rename = "type")] // Use rename instead of r#
    pub topic_type: String,
    pub serialization_format: String,
    // Use custom deserialization for the nested YAML string
    #[serde(deserialize_with = "deserialize_qos_profiles")]
    pub offered_qos_profiles: Vec<QosProfile>,
}

// --- QoS Profile Structs (New) ---

#[derive(Deserialize, Debug)]
pub struct QosProfile {
    // Using i32 for enum-like values, could be mapped to actual enums if needed
    pub history: i32,
    pub depth: i32,
    pub reliability: i32,
    pub durability: i32,
    pub deadline: QosTime,
    pub lifespan: QosTime,
    pub liveliness: i32,
    pub liveliness_lease_duration: QosTime,
    pub avoid_ros_namespace_conventions: bool,
}

#[derive(Deserialize, Debug, Clone, Copy)] // Added Clone, Copy
pub struct QosTime {
    // Using i64/u32 based on typical ROS time representations and potential values
    pub sec: i64,
    pub nsec: u32,
}

// --- File Information Structs ---

#[derive(Deserialize, Debug)]
pub struct File {
    pub path: String,
    pub starting_time: NanosecondsSinceEpoch,
    pub duration: Nanoseconds,
    pub message_count: i64, // Changed to i64
}

/// Custom deserializer for the `offered_qos_profiles` field.
/// It expects a string containing YAML, which it then parses using `serde_yaml`.
fn deserialize_qos_profiles<'de, D>(deserializer: D) -> Result<Vec<QosProfile>, D::Error>
where
    D: Deserializer<'de>,
{
    struct QosProfilesVisitor;

    impl<'de> de::Visitor<'de> for QosProfilesVisitor {
        type Value = Vec<QosProfile>;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("a string containing YAML data for QoS profiles")
        }

        // Process the string value obtained from the YAML field
        fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            // Use serde_yaml to parse the *content* of the string
            serde_yml::from_str(value).map_err(de::Error::custom)
        }

        // Handle cases where the YAML might surprisingly contain a sequence directly
        fn visit_seq<A>(self, seq: A) -> Result<Self::Value, A::Error>
        where
            A: de::SeqAccess<'de>,
        {
            serde::Deserialize::deserialize(de::value::SeqAccessDeserializer::new(seq))
        }
    }

    deserializer.deserialize_any(QosProfilesVisitor)
}

fn validate_support(data: &Metadata) -> anyhow::Result<()> {
    if data.rosbag2_bagfile_information.storage_identifier != "mcap" {
        return Err(anyhow!("Only supporting mcap storage. Please convert it."));
    }

    for twm in data
        .rosbag2_bagfile_information
        .topics_with_message_count
        .iter()
    {
        if twm.topic_metadata.serialization_format != "cdr" {
            return Err(anyhow!(
                "Only supporting cdr serialisation. Please convert it."
            ));
        }
    }

    if data.rosbag2_bagfile_information.files.len() > 1 {
        return Err(anyhow!("More than one file in an mcap bagfile."));
    }

    Ok(())
}

#[derive(Default, Debug)]
pub struct Bagfile {
    bagfile_name: Option<std::path::PathBuf>,
    cursor: usize,
    buffer: Option<Mmap>,
    metadata: Option<Metadata>,
}

// TODO change in rlc lib.rs to use this here?
// #[derive(Debug, PartialEq, Clone)]
// pub enum PlayKindUnited {
//     SensorCount {
//         sensor: SensorType,
//         count: usize,
//     },
//     UntilSensorCount {
//         sending: SensorType,
//         until_sensor: SensorType,
//         until_count: usize,
//     },
//     UntilTime {
//         sending: SensorType,
//         duration: std::time::Duration,
//     },
// }

// #[derive(Debug, PartialEq, Clone)]
// pub enum SensorType {
//     Lidar { topic: String },
//     Imu { topic: String },
//     Mixed { topics: Vec<String> },
// }

pub enum BagMsg {
    Cloud(PointCloud2Msg),
    Imu(ImuMsg),
}

#[derive(Debug, PartialEq, Clone)]
pub enum PlayKindUnitedRich {
    SensorCount {
        sensor: SensorTypeRich,
        count: PlayCount,
        trigger: Option<PlayTrigger>,
    },
    UntilSensorCount {
        sending: SensorTypeRich,
        until_sensor: SensorTypeRich,
        until_count: PlayCount,
        trigger: Option<PlayTrigger>,
    },
}

impl PlayKindUnitedRich {
    pub fn with_topic(src: PlayKindUnited, topics: &[&str]) -> Self {
        match src {
            PlayKindUnited::SensorCount {
                sensor,
                count,
                trigger,
            } => PlayKindUnitedRich::SensorCount {
                sensor: SensorTypeRich::with_topic(&sensor, topics),
                count,
                trigger,
            },
            PlayKindUnited::UntilSensorCount {
                sending,
                until_sensor,
                until_count,
                trigger,
            } => PlayKindUnitedRich::UntilSensorCount {
                sending: SensorTypeRich::with_topic(&sending, topics),
                until_sensor: SensorTypeRich::with_topic(&until_sensor, topics),
                until_count,
                trigger,
            },
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum SensorTypeRich {
    Lidar { topic: String },
    Imu { topic: String },
    Mixed { topics: Vec<String> },
}

impl SensorTypeRich {
    pub fn with_topic(src: &SensorType, topics: &[&str]) -> Self {
        match src {
            SensorType::Lidar => SensorTypeRich::Lidar {
                topic: (*topics[0]).to_owned(),
            },
            SensorType::Imu => SensorTypeRich::Imu {
                topic: (*topics[1]).to_owned(),
            },
            SensorType::Mixed => SensorTypeRich::Mixed {
                topics: topics
                    .into_iter()
                    .map(|a| (*a).to_owned())
                    .collect::<Vec<_>>(),
            },
        }
    }
}

fn collect_until_count(
    iter: impl Iterator<Item = core::result::Result<Message<'static>, McapError>>,
    metadata: &Metadata,
    send_sensor: &SensorTypeRich,
    until_sensor: Option<&SensorTypeRich>,
    until_count: usize,
) -> anyhow::Result<Vec<BagMsg>> {
    let mut counter = 0;
    let mut msgs = Vec::new();

    let stop_topics = until_sensor.map(|st| match st {
        SensorTypeRich::Lidar { topic } => vec![topic.clone()],
        SensorTypeRich::Imu { topic } => vec![topic.clone()],
        SensorTypeRich::Mixed { topics } => topics.clone(),
    });

    let mut iter = iter.peekable();

    loop {
        match iter.peek() {
            Some(msg) => {
                let msg = msg
                    .as_ref()
                    .map_err(|e| anyhow!("Could not read mcap: {e}"))?;

                match &stop_topics {
                    Some(stop_topics) => {
                        if stop_topics.contains(&msg.channel.topic) {
                            counter += 1;
                            if counter >= until_count {
                                return Ok(msgs);
                            }
                        }
                    }
                    None => {}
                };

                let msgtype = metadata
                    .get_topic_meta(&msg.channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let send_topics = match &send_sensor {
                    SensorTypeRich::Lidar { topic } => vec![topic.clone()],
                    SensorTypeRich::Imu { topic } => vec![topic.clone()],
                    SensorTypeRich::Mixed { topics } => topics.clone(),
                };

                if send_topics.contains(&msgtype.name) {
                    // TODO compression very likely not supported since no dep. maybe still needs rustdds? how to use the deserializer? or just decomp manually before deserialize?
                    let len_before = msgs.len();
                    match send_sensor {
                        SensorTypeRich::Lidar { topic: _ } => {
                            let dec = cdr::deserialize::<PointCloud2>(&msg.data)
                                .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            msgs.push(BagMsg::Cloud(dec.into()));
                        }
                        SensorTypeRich::Imu { topic: _ } => {
                            let dec = cdr::deserialize::<Imu>(&msg.data)
                                .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                            msgs.push(BagMsg::Imu(dec.into()));
                        }
                        SensorTypeRich::Mixed { topics: _ } => match msgtype.topic_type.as_str() {
                            "sensor_msgs/msg/Imu" => {
                                let dec = cdr::deserialize::<Imu>(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                                msgs.push(BagMsg::Imu(dec.into()));
                            }
                            "sensor_msgs/msg/PointCloud2" => {
                                let dec = cdr::deserialize::<PointCloud2>(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                                msgs.push(BagMsg::Cloud(dec.into()));
                            }
                            _ => {}
                        },
                    }

                    if stop_topics.is_none() && len_before != msgs.len() {
                        counter += 1;
                    }

                    if counter >= until_count {
                        return Ok(msgs);
                    }
                }
            }
            None => break,
        }

        let _ = iter.next(); // consume
    }

    return Ok(msgs);
}

// collect from a specific type until a time has passed inside the bagfile time span
// TODO if not given variable but a time, use that time as frequency and just publish it without waiting for a variable
fn collect_until_time(
    iter: impl Iterator<Item = core::result::Result<Message<'static>, McapError>>,
    metadata: &Metadata,
    send_sensor: &SensorTypeRich,
    until_time: &std::time::Duration,
) -> anyhow::Result<Vec<BagMsg>> {
    let dur = until_time.as_nanos();
    let mut start_time = None;
    let mut msgs = Vec::new();

    let mut iter = iter.peekable();

    loop {
        match iter.peek() {
            Some(msg) => {
                let msg = msg
                    .as_ref()
                    .map_err(|e| anyhow!("Could not read mcap: {e}"))?;
                if let Some(tstart) = start_time {
                    if tstart + dur < msg.publish_time as u128 {
                        return Ok(msgs);
                    }
                } else {
                    start_time.replace(msg.publish_time as u128);
                }

                let msgtype = metadata
                    .get_topic_meta(&msg.channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let send_topics = match &send_sensor {
                    SensorTypeRich::Lidar { topic } => vec![topic.clone()],
                    SensorTypeRich::Imu { topic } => vec![topic.clone()],
                    SensorTypeRich::Mixed { topics } => topics.clone(),
                };

                if send_topics.contains(&msgtype.name) {
                    // TODO compression very likely not supported since no dep. maybe still needs rustdds? how to use the deserializer? or just decomp manually before deserialize?
                    match send_sensor {
                        SensorTypeRich::Lidar { topic: _ } => {
                            let dec = cdr::deserialize::<PointCloud2>(&msg.data)
                                .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                            msgs.push(BagMsg::Cloud(dec.into()));
                        }
                        SensorTypeRich::Imu { topic: _ } => {
                            let dec = cdr::deserialize::<Imu>(&msg.data)
                                .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                            msgs.push(BagMsg::Imu(dec.into()));
                        }
                        SensorTypeRich::Mixed { topics: _ } => match msgtype.topic_type.as_str() {
                            "sensor_msgs/msg/Imu" => {
                                let dec = cdr::deserialize::<Imu>(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                                msgs.push(BagMsg::Imu(dec.into()));
                            }
                            "sensor_msgs/msg/PointCloud2" => {
                                let dec = cdr::deserialize::<PointCloud2>(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                                msgs.push(BagMsg::Cloud(dec.into()));
                            }
                            _ => {}
                        },
                    }
                }
            }
            None => break,
        }

        let _ = iter.next(); // consume
    }

    return Ok(msgs);
}

impl Bagfile {
    // TODO handle trigger and counts coorectly
    pub fn next(&mut self, kind: &PlayKindUnitedRich) -> anyhow::Result<Vec<BagMsg>> {
        match self.buffer.as_ref() {
            Some(buffer) => {
                let stream = mcap::MessageStream::new(buffer)?;
                let iter = stream.skip(self.cursor);
                let metadata = self.metadata.as_ref().expect("metadata set with buffer");
                match kind {
                    PlayKindUnitedRich::SensorCount {
                        sensor,
                        count,
                        trigger,
                    } => collect_until_count(iter, metadata, sensor, None, 0),
                    PlayKindUnitedRich::UntilSensorCount {
                        sending,
                        until_sensor,
                        until_count,
                        trigger,
                    } => collect_until_count(iter, metadata, sending, Some(until_sensor), 1),
                }
            }
            None => Err(anyhow!("Not yet initialized.")),
        }
    }

    pub fn reset(&mut self, path: Option<impl AsRef<Path>>) -> anyhow::Result<()> {
        let path = path
            .map(|p| p.as_ref().to_path_buf())
            .or(self.bagfile_name.clone());
        let changed_path_or_new = match (&path, &self.bagfile_name) {
            (None, None) => false,
            (None, Some(_)) => false,
            (Some(_), None) => true,
            (Some(npath), Some(old_path)) => {
                let path = npath.to_path_buf();
                old_path != &path
            }
        };
        if changed_path_or_new {
            let path = path.unwrap();
            let metapath = path.join("metadata.yaml");

            let metadata_contents = fs::read_to_string(metapath).unwrap();
            let bag_info: Metadata = serde_yml::from_str(&metadata_contents)?;
            validate_support(&bag_info)?;
            let mcap_file = bag_info
                .rosbag2_bagfile_information
                .files
                .first()
                .expect("Validated after deserialisation.");

            let mcap_path = path.join(&mcap_file.path);
            let fd = fs::File::open(mcap_path).context("Couldn't open MCAP file")?;
            let buffer = unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")?;
            self.buffer = Some(buffer);
            self.metadata = Some(bag_info);
        }

        self.cursor = 0;
        Ok(())

        // let bag_duration = std::time::Duration::from_nanos(
        //     bag_info.rosbag2_bagfile_information.duration.nanoseconds,
        // );

        // let bag_start_time_since_epoch_duration = Duration::from_nanos(
        //     bag_info
        //         .rosbag2_bagfile_information
        //         .starting_time
        //         .nanoseconds_since_epoch,
        // );
        // let bag_start_time = std::time::UNIX_EPOCH
        //     .checked_add(bag_start_time_since_epoch_duration)
        //     .ok_or_else(|| anyhow!("nanoseconds_since_epoch unreasonable large."))?;

        // let mapped_b = borrow_with_lifetime(&mapped);
        // let stream = mcap::MessageStream::new(&mapped)?;

        // for message in mcap::MessageStream::new(&mapped)? {
        //     let msg = message?;

        //     dbg!(msg.channel, msg.sequence, msg.publish_time);
        //     // match msg.channel {}
        //     // Or whatever else you'd like to do...
        // }
    }
}

// fn read(path: impl AsRef<Path>) -> anyhow::Result<Bagfile> {
//     let path = path.as_ref().to_path_buf();
//     let metapath = path.join("metadata.yaml");

//     let metadata_contents = fs::read_to_string(metapath).unwrap();
//     let bag_info: Metadata = serde_yml::from_str(&metadata_contents)?;
//     validate_support(&bag_info)?;

//     let bag_duration =
//         std::time::Duration::from_nanos(bag_info.rosbag2_bagfile_information.duration.nanoseconds);

//     let bag_start_time_since_epoch_duration = Duration::from_nanos(
//         bag_info
//             .rosbag2_bagfile_information
//             .starting_time
//             .nanoseconds_since_epoch,
//     );
//     let bag_start_time = std::time::UNIX_EPOCH
//         .checked_add(bag_start_time_since_epoch_duration)
//         .ok_or_else(|| anyhow!("nanoseconds_since_epoch unreasonable large."))?;

//     let mcap_file = bag_info
//         .rosbag2_bagfile_information
//         .files
//         .first()
//         .expect("Validated after deserialisation.");

//     let mcap_path = path.join(&mcap_file.path);
//     let fd = fs::File::open(mcap_path).context("Couldn't open MCAP file")?;
//     let mapped = unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")?;

//     let stream = mcap::MessageStream::new(&mapped)?;

//     // for message in mcap::MessageStream::new(&mapped)? {
//     //     let msg = message?;

//     //     dbg!(msg.channel, msg.sequence, msg.publish_time);
//     //     // match msg.channel {}
//     //     // Or whatever else you'd like to do...
//     // }
//     Ok(Bagfile {
//         bagfile_name: Some(mcap_path),
//         cursor: 0,
//         buffer: None,
//         metadata: None,
//     })
// }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bag_read_simple() {
        // TODO add trigger
        let path = "../dlg_cut";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok());

        let clouds = bag.next(&PlayKindUnitedRich::SensorCount {
            sensor: SensorTypeRich::Lidar {
                topic: "/ouster/points".to_owned(),
            },
            count: PlayCount::Amount(1),
            trigger: None,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);
    }

    #[test]
    fn bag_read_time() {
        let path = "../dlg_cut";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok());

        let clouds = bag.next(&PlayKindUnitedRich::SensorCount {
            sensor: SensorTypeRich::Lidar {
                topic: "/ouster/points".to_owned(),
            },
            trigger: None,
            count: PlayCount::TimeRelativeMs(50),
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);
    }
}
