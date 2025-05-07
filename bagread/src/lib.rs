use std::{fs, path::Path};

use anyhow::{Context, Result, anyhow};
use mcap::{McapError, Message};
use memmap2::Mmap;
use nalgebra::{UnitQuaternion, Vector3};
use rlc::{AbsTimeRange, PlayCount, PlayMode, PlayTrigger, SensorType};
pub use ros_pointcloud2::PointCloud2Msg;
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
    pub message_count: i64,
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

        fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            serde_yml::from_str(value).map_err(de::Error::custom)
        }

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

#[derive(Debug)]
pub struct BagMsg {
    topic: String,
    msg_type: String,
    data: SensorTypeMapped,
}

#[derive(Debug, PartialEq, Clone)]
pub enum PlayKindUnitedRich {
    SensorCount {
        sensor: Vec<AnySensor>,
        count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
    UntilSensorCount {
        sending: Vec<AnySensor>,
        until_sensor: Vec<AnySensor>,
        until_count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
}

// impl PlayKindUnitedRich {
//     pub fn with_topic(src: PlayKindUnited, topics: &[&str]) -> Self {
//         match src {
//             PlayKindUnited::SensorCount {
//                 sensor,
//                 count,
//                 trigger,
//                 play_mode,
//             } => PlayKindUnitedRich::SensorCount {
//                 sensor: SensorTypeRich::with_topic(&sensor, topics),
//                 count,
//                 trigger,
//                 play_mode,
//             },
//             PlayKindUnited::UntilSensorCount {
//                 sending,
//                 until_sensor,
//                 until_count,
//                 trigger,
//                 play_mode,
//             } => PlayKindUnitedRich::UntilSensorCount {
//                 sending: SensorTypeRich::with_topic(&sending, topics),
//                 until_sensor: SensorTypeRich::with_topic(&until_sensor, topics),
//                 until_count,
//                 trigger,
//                 play_mode,
//             },
//         }
//     }
// }

#[derive(Debug, Clone)]
pub enum SensorTypeMapped {
    Lidar(PointCloud2Msg),
    Imu(ImuMsg),
    Any,
}

#[derive(Debug, PartialEq, Clone)]
pub enum SensorIdentification {
    Topics(Vec<String>), // no type given, compare by topics
    Type(String),        // type given but no topics, check for types
    TopicsAndType {
        topics: Vec<String>,
        msg_type: String,
    }, // both given but only use topics for collect until because it is more fine grained
}

#[derive(Debug, PartialEq, Clone)]
pub struct AnySensor {
    pub name: String, // custom name given by config
    pub id: SensorIdentification,
    pub short: Option<String>, // to be used while parsing to not write so much
}

// impl SensorTypeRich {
//     pub fn with_topic(src: &SensorType, topics: &[&str]) -> Self {
//         match src {
//             SensorType::Lidar => SensorTypeRich::Lidar {
//                 topic: (*topics[0]).to_owned(),
//             },
//             SensorType::Imu => SensorTypeRich::Imu {
//                 topic: (*topics[1]).to_owned(),
//             },
//             SensorType::Mixed => SensorTypeRich::Mixed {
//                 topics: topics
//                     .into_iter()
//                     .map(|a| (*a).to_owned())
//                     .collect::<Vec<_>>(),
//             },
//         }
//     }
// }

fn collect_until(
    iter: impl Iterator<Item = core::result::Result<Message<'static>, McapError>>,
    cursor: &mut usize,
    metadata: &Metadata,
    send_sensor: &Vec<AnySensor>,
    until_sensor: Option<(&Vec<AnySensor>, &PlayCount)>,
    until: Option<&PlayCount>,
) -> anyhow::Result<Vec<(u64, BagMsg)>> {
    let mut rel_since_begin = 0;
    let mut until_sensor_counter = 0;
    let mut msgs = Vec::new();
    let mut start_time = None;

    let skip = *cursor;
    let mut skip_start_time = None;
    let mut iter = iter
        .enumerate()
        .skip_while(|(i, s)| {
            match until {
                Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, _))))
                | Some(PlayCount::TimeRangeMs(AbsTimeRange::UpperOpen(min))) => {
                    let min = (*min as u128) * 1_000_000;
                    if let Ok(msg) = s {
                        let msg_time = msg.publish_time as u128;
                        if let Some(tstart) = skip_start_time {
                            // we haven't reached our start point yet, so skip this one
                            msg_time < tstart + min
                        } else {
                            // edge case: span starts from beginning so we can't compare internal msg time with a previous ts since the range ms is relative time from bagfile start and msg_time is absolute
                            if *i == 0 && min == 0 {
                                false
                            } else {
                                skip_start_time.replace(msg.publish_time as u128);
                                true
                            }
                        }
                    } else {
                        // failed reading mcap, propagate error to iterator peek later
                        false
                    }
                }
                _ => {
                    let skipped = *i < skip;
                    skipped
                }
            }
        })
        .map(|(_, el)| el)
        .peekable();

    loop {
        match iter.peek() {
            Some(msg) => {
                let msg = msg
                    .as_ref()
                    .map_err(|e| anyhow!("Could not read mcap: {e}"))?;

                match until {
                    Some(PlayCount::TimeRelativeMs(rel_dur)) => {
                        let rel_dur = rel_dur * 1_000_000;
                        if let Some(tstart) = start_time {
                            if msg.publish_time > tstart + rel_dur {
                                return Ok(msgs);
                            }
                        } else {
                            start_time.replace(msg.publish_time);
                        }
                    }
                    Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, max)))) => {
                        let min = min * 1_000_000;
                        let max = max * 1_000_000;
                        let rel_dur = max - min;
                        if let Some(tstart) = start_time {
                            if tstart + rel_dur < msg.publish_time {
                                return Ok(msgs);
                            }
                        } else {
                            start_time.replace(msg.publish_time);
                        }
                    }
                    Some(PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(max))) => {
                        let max = max * 1_000_000;
                        if let Some(tstart) = start_time {
                            if tstart + max < msg.publish_time {
                                return Ok(msgs);
                            }
                        } else {
                            start_time.replace(msg.publish_time);
                        }
                    }
                    Some(PlayCount::Amount(count)) => {
                        if rel_since_begin >= *count {
                            return Ok(msgs);
                        }
                    }
                    _ => {}
                }

                match &until_sensor {
                    Some((until_sensors, pc)) => {
                        for us in *until_sensors {
                            let pass = match &us.id {
                                SensorIdentification::Topics(items) => {
                                    items.contains(&msg.channel.topic)
                                }
                                SensorIdentification::Type(t) => {
                                    t.as_str() == msg.channel.topic.as_str()
                                }
                                SensorIdentification::TopicsAndType {
                                    topics,
                                    msg_type: _,
                                } => topics.contains(&msg.channel.topic),
                            };

                            if pass {
                                until_sensor_counter += 1;
                                match pc {
                                    PlayCount::Amount(uc) => {
                                        if until_sensor_counter >= *uc {
                                            return Ok(msgs);
                                        }
                                    }
                                    _ => {
                                        return Err(anyhow!(
                                            "Can only mix until_sensor with an amount. Timings are not specified to a sensor, use them in the stop criteria without a sensor."
                                        ));
                                    }
                                }
                            }
                        }
                    }
                    None => {}
                }

                let msgtype = metadata
                    .get_topic_meta(&msg.channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let pass = send_sensor.iter().any(|a| match &a.id {
                    SensorIdentification::Topics(items) => items.contains(&msg.channel.topic),
                    SensorIdentification::Type(t) => t.as_str() == msg.channel.topic.as_str(),
                    SensorIdentification::TopicsAndType {
                        topics,
                        msg_type: _,
                    } => topics.contains(&msg.channel.topic),
                });
                let len_before = msgs.len();

                if pass {
                    let enc = BagMsg::Any {
                        topic: msgtype.name.clone(),
                        msg_type: msgtype.topic_type.clone(),
                        data: msg.data.to_vec(),
                    };
                    msgs.push((msg.publish_time, enc));
                }
                if len_before != msgs.len() {
                    rel_since_begin += 1;
                }
            }
            None => break,
        }

        let _ = iter.next(); // consume
        *cursor += 1;
    }

    return Ok(msgs);
}

impl Bagfile {
    pub fn next(&mut self, kind: &PlayKindUnitedRich) -> anyhow::Result<Vec<(u64, BagMsg)>> {
        match self.buffer.as_ref() {
            Some(buffer) => {
                let stream = mcap::MessageStream::new(buffer)?;
                let metadata = self.metadata.as_ref().expect("metadata set with buffer");
                match kind {
                    PlayKindUnitedRich::SensorCount {
                        sensor,
                        count,
                        trigger: _,
                        play_mode: _,
                    } => collect_until(
                        stream,
                        &mut self.cursor,
                        metadata,
                        sensor,
                        None,
                        Some(count),
                    ),
                    PlayKindUnitedRich::UntilSensorCount {
                        sending,
                        until_sensor,
                        until_count,
                        trigger: _,
                        play_mode: _,
                    } => collect_until(
                        stream,
                        &mut self.cursor,
                        metadata,
                        sending,
                        Some((until_sensor, until_count)),
                        None,
                    ),
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

            let metadata_contents = fs::read_to_string(metapath)?;
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bag_read_simple() {
        let path = "../dlg_cut";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok());

        let clouds = bag.next(&PlayKindUnitedRich::SensorCount {
            sensor: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topics(vec!["/ouster/points".to_owned()]),
                short: None,
            }],
            count: PlayCount::Amount(1),
            trigger: None,
            play_mode: PlayMode::Fix,
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
            sensor: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topics(vec!["/ouster/points".to_owned()]),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRelativeMs(50),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);

        let clouds = bag.next(&PlayKindUnitedRich::SensorCount {
            sensor: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topics(vec!["/ouster/points".to_owned()]),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRangeMs(rlc::AbsTimeRange::Closed((50, 100))),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);

        let clouds = bag.next(&PlayKindUnitedRich::SensorCount {
            sensor: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topics(vec!["/ouster/points".to_owned()]),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRangeMs(rlc::AbsTimeRange::Closed((0, 100))),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 2);
    }
}
