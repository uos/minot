use std::time::Duration;
use std::{fs, path::Path};

use anyhow::{Context, Result, anyhow};
use camino::Utf8Path;
use cdr::{CdrLe, Infinite};
use memmap2::Mmap;
use nalgebra::{UnitQuaternion, Vector3};
pub use ros_pointcloud2::PointCloud2Msg;
use ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2;
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

#[derive(Default)]
pub struct Bagfile {
    bagfile_name: Option<std::path::PathBuf>,
    cursor: usize,
    buffer: Option<Mmap>,
    metadata: Option<Metadata>,
}

// TODO change in rlc lib.rs to use this here?
#[derive(Debug, PartialEq, Clone)]
pub enum PlayKindUnited {
    SensorCount {
        sensor: SensorType,
        count: usize,
    },
    UntilSensorCount {
        sending: SensorType,
        until_sensor: SensorType,
        until_count: usize,
    },
    UntilTime {
        sending: SensorType,
        duration: std::time::Duration,
    },
}

#[derive(Debug, PartialEq, Clone)]
pub enum SensorType {
    Lidar { topic: String },
    Imu { topic: String },
    Mixed { topics: Vec<String> },
}

pub enum BagMsg {
    Cloud(ros_pointcloud2::PointCloud2Msg),
    Imu(ImuMsg),
}

impl Bagfile {
    pub fn next(&mut self, kind: PlayKindUnited) -> anyhow::Result<Vec<BagMsg>> {
        match self.buffer.as_ref() {
            Some(buffer) => {
                let stream = mcap::MessageStream::new(buffer)?;
                let iter = stream.skip(self.cursor);
                let metadata = self.metadata.as_ref().expect("metadata set with buffer");
                Ok(match kind {
                    PlayKindUnited::SensorCount { sensor, count } => match sensor {
                        SensorType::Lidar { topic } => {
                            let mut lidar_counter = 0;
                            let mut msgs = Vec::new();

                            for msg in iter {
                                if lidar_counter >= count {
                                    return Ok(msgs);
                                }
                                let msg = msg?;
                                if msg.channel.topic == topic {
                                    let msgtype = metadata
                                        .get_topic_meta(&msg.channel.topic)
                                        .ok_or(anyhow!("Topic does not exist."))?;
                                    if msgtype.topic_type != "sensor_msgs/msg/PointCloud2" {
                                        return Err(anyhow!("Lidar topic must be PointCloud2"));
                                    }

                                    lidar_counter += 1;

                                    // TODO compression very likely not supported since no dep. maybe still needs rustdds? how to use the deserializer? or just decomp manually before deserialize?
                                    let dec = cdr::deserialize::<PointCloud2>(&msg.data)
                                        .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;

                                    msgs.push(BagMsg::Cloud(dec.into()));
                                }
                            }

                            return Ok(msgs);
                        }
                        _ => {
                            vec![]
                        }
                    },
                    PlayKindUnited::UntilSensorCount {
                        sending,
                        until_sensor,
                        until_count,
                    } => todo!(),
                    PlayKindUnited::UntilTime { sending, duration } => todo!(),
                })
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

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let path = "/Users/stelzo/projects/liolink/dlg_cut";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok());

        let clouds = bag
            .next(PlayKindUnited::SensorCount {
                sensor: SensorType::Lidar {
                    topic: "/ouster/points".to_owned(),
                },
                count: 1,
            })
            .unwrap();

        // assert!(clouds.is_ok());
        // let clouds = clouds.unwrap();

        // assert_eq!(clouds.len(), 1);
    }
}
