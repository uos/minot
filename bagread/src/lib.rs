use std::str::FromStr;
use std::{fs, path::Path};

use anyhow::{Context, Result, anyhow};
use mcap::{McapError, Message};
use memmap2::Mmap;
use qos::{
    RmwQosDurabilityPolicy, RmwQosHistoryPolicy, RmwQosLivelinessPolicy, RmwQosReliabilityPolicy,
};
use rkyv::Archive;
use rlc::{
    AbsTimeRange, AnySensor, IMU_ROS2_TYPE, POINTCLOUD_ROS2_TYPE, PlayCount, PlayKindUnitedPass3,
    SensorIdentification, SensorType,
};
pub use ros_pointcloud2::PointCloud2Msg;
use ros2_client::ros2::policy::{Durability, History, Reliability};
use ros2_client::ros2::{self, Duration};
// use ros2_interfaces_jazzy::sensor_msgs::msg::{Imu, PointCloud2};
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{Imu, PointCloud2};
use serde::{Deserialize, Serialize};
use serde::{Deserializer, de};
use std::fmt;

pub mod qos;
pub use Qos::*;

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

#[derive(Archive, Deserialize, Debug, Serialize, Clone, rkyv::Deserialize, rkyv::Serialize)]
pub struct QosProfile {
    pub history: String,
    pub depth: i32,
    pub reliability: String,
    pub durability: String,
    pub deadline: QosTime,
    pub lifespan: QosTime,
    pub liveliness: String,
    pub liveliness_lease_duration: QosTime,
    pub avoid_ros_namespace_conventions: bool,
}

#[derive(
    Deserialize, Serialize, rkyv::Serialize, rkyv::Deserialize, Archive, Debug, Clone, Copy,
)] // Added Clone, Copy
pub struct QosTime {
    pub sec: u64,
    pub nsec: u64,
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

#[derive(Clone, Debug, rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)]
pub struct BagMsg {
    pub topic: String,
    pub msg_type: String,
    pub data: SensorTypeMapped,
    pub qos: Option<Qos>,
}

#[derive(Clone, Debug, rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)]
pub enum SensorTypeMapped {
    Lidar(PointCloud2),
    Imu(Imu),
    Any(Vec<u8>),
}

// pub fn detect_endianness_from_header<R>(mut reader: R) -> anyhow::Result<Endian>
// where
//     R: std::io::Read,
// {
//     const ENCAPSULATION_HEADER_SIZE: usize = 4;
//     let mut header_buffer = [0u8; ENCAPSULATION_HEADER_SIZE];

//     match reader.read_exact(&mut header_buffer) {
//         Ok(_) => match header_buffer.get(1) {
//             Some(&0) | Some(&2) => Ok(Endian::Big),
//             Some(&1) | Some(&3) => Ok(Endian::Little),
//             Some(_) => Err(anyhow!("Invalid Header")),
//             None => Err(anyhow!("Unexpected eof")),
//         },
//         Err(ref e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
//             Err(anyhow!("Unexpected eof"))
//         }
//         Err(e) => Err(e.into()),
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
                                SensorIdentification::Topic(items) => {
                                    items.as_str() == &msg.channel.topic
                                }
                                SensorIdentification::Type(t) => t.is(msg.channel.topic.as_str()),
                                SensorIdentification::TopicAndType { topic, msg_type: _ } => {
                                    topic.as_str() == &msg.channel.topic
                                }
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

                let topic_meta = metadata
                    .get_topic_meta(&msg.channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let mut send_type = SensorType::Any;
                let mut pass = false;
                for sensor in send_sensor.iter() {
                    let (n_pass, n_send_type) = match &sensor.id {
                        SensorIdentification::Topic(item) => {
                            (item.as_str() == &msg.channel.topic, SensorType::Any)
                        }
                        SensorIdentification::Type(t) => {
                            (t.is(msg.channel.topic.as_str()), t.clone())
                        }
                        SensorIdentification::TopicAndType { topic, msg_type: t } => {
                            (topic.as_str() == &msg.channel.topic, t.clone())
                        }
                    };

                    send_type = n_send_type;
                    pass = n_pass;
                    if n_pass {
                        break;
                    }
                }

                let len_before = msgs.len();

                if pass {
                    let data = match send_type {
                        SensorType::Lidar => {
                            let dec: ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2 =
                                cdr::deserialize(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            let data: PointCloud2 = unsafe { std::mem::transmute(dec) };

                            SensorTypeMapped::Lidar(data)
                        }
                        SensorType::Imu => {
                            let dec: ros2_interfaces_jazzy::sensor_msgs::msg::Imu =
                                cdr::deserialize(&msg.data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            let data: Imu = unsafe { std::mem::transmute(dec) };

                            SensorTypeMapped::Imu(data)
                        }
                        SensorType::Mixed => match topic_meta.topic_type.as_str() {
                            POINTCLOUD_ROS2_TYPE => {
                                let dec: ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2 =
                                    cdr::deserialize(&msg.data)
                                        .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                                let data: PointCloud2 = unsafe { std::mem::transmute(dec) };

                                SensorTypeMapped::Lidar(data)
                            }
                            IMU_ROS2_TYPE => {
                                let dec: ros2_interfaces_jazzy::sensor_msgs::msg::Imu =
                                    cdr::deserialize(&msg.data)
                                        .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                                let data: Imu = unsafe { std::mem::transmute(dec) };

                                SensorTypeMapped::Imu(data)
                            }
                            _ => SensorTypeMapped::Any(msg.data.to_vec()),
                        },
                        SensorType::Any => SensorTypeMapped::Any(msg.data.to_vec()),
                    };
                    let qos = if let Some(last) = topic_meta.offered_qos_profiles.last() {
                        // There are more than one here but seem similar, idk... maybe every publisher can set a different one, but which one we pub?
                        // TODO override with settings from ratslang file
                        Some(Qos::Cutom(last.clone()))
                    } else {
                        None
                    };
                    let enc = BagMsg {
                        topic: topic_meta.name.clone(),
                        msg_type: topic_meta.topic_type.clone(),
                        data,
                        qos,
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

#[derive(Clone, Debug, Default, Archive, rkyv::Serialize, rkyv::Deserialize)]
pub enum Qos {
    Sensor,
    #[default]
    SystemDefault,
    Cutom(QosProfile),
}

impl TryFrom<String> for Qos {
    type Error = anyhow::Error;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        match value.as_str() {
            "sensor" => Ok(Qos::Sensor),
            "system_default" => Ok(Qos::SystemDefault),
            _ => Err(anyhow!("Invalid QoS value")),
        }
    }
}

impl TryFrom<crate::Qos> for ros2::QosPolicies {
    type Error = anyhow::Error;
    fn try_from(value: crate::Qos) -> Result<Self, Self::Error> {
        Ok(match value {
            Qos::Sensor => ros2::QosPolicyBuilder::new()
                .durability(ros2::policy::Durability::Volatile)
                .deadline(ros2::policy::Deadline(ros2::Duration::INFINITE))
                .ownership(ros2::policy::Ownership::Shared)
                .reliability(ros2::policy::Reliability::BestEffort)
                .history(ros2::policy::History::KeepLast { depth: 5 })
                .lifespan(ros2::policy::Lifespan {
                    duration: ros2::Duration::INFINITE,
                })
                .build(),
            Qos::SystemDefault => ros2::QosPolicyBuilder::new()
                .durability(ros2::policy::Durability::Volatile)
                .deadline(ros2::policy::Deadline(ros2::Duration::INFINITE))
                .ownership(ros2::policy::Ownership::Shared)
                .reliability(ros2::policy::Reliability::Reliable {
                    max_blocking_time: ros2::Duration::from_millis(100),
                })
                .history(ros2::policy::History::KeepLast { depth: 1 })
                .lifespan(ros2::policy::Lifespan {
                    duration: ros2::Duration::INFINITE,
                })
                .build(),
            Qos::Cutom(q) => {
                let deadline_dur = std::time::Duration::from_secs(q.deadline.sec)
                    + std::time::Duration::from_nanos(q.deadline.nsec);
                let deadline = ros2::policy::Deadline(Duration::from_std(deadline_dur));
                let lifespan_dur = std::time::Duration::from_secs(q.lifespan.sec)
                    + std::time::Duration::from_nanos(q.lifespan.nsec);
                let lifespan = ros2::policy::Lifespan {
                    duration: Duration::from_std(lifespan_dur),
                };

                let reliability = match RmwQosReliabilityPolicy::from_str(&q.reliability)? {
                    RmwQosReliabilityPolicy::SystemDefault => Reliability::Reliable {
                        max_blocking_time: Duration::INFINITE,
                    },
                    RmwQosReliabilityPolicy::Reliable => Reliability::Reliable {
                        max_blocking_time: Duration::INFINITE,
                    },
                    RmwQosReliabilityPolicy::BestEffort => Reliability::BestEffort,
                    RmwQosReliabilityPolicy::Unknown => {
                        return Err(anyhow!("Unknown reliability"));
                    }
                    RmwQosReliabilityPolicy::BestAvailable => {
                        // TODO maybe warn here, qos not found
                        Reliability::Reliable {
                            max_blocking_time: Duration::INFINITE,
                        }
                    }
                };

                let durability = match RmwQosDurabilityPolicy::from_str(&q.durability)? {
                    RmwQosDurabilityPolicy::SystemDefault => Durability::TransientLocal,
                    RmwQosDurabilityPolicy::TransientLocal => Durability::TransientLocal,
                    RmwQosDurabilityPolicy::Volatile => Durability::Volatile,
                    RmwQosDurabilityPolicy::Unknown => {
                        return Err(anyhow!("Durability set to unknown."));
                    }
                    RmwQosDurabilityPolicy::BestAvailable => Durability::TransientLocal,
                };

                let history = match RmwQosHistoryPolicy::from_str(&q.history)? {
                    RmwQosHistoryPolicy::SystemDefault => History::KeepLast { depth: q.depth },
                    RmwQosHistoryPolicy::KeepLast => History::KeepLast { depth: q.depth },
                    RmwQosHistoryPolicy::KeepAll => History::KeepAll,
                    RmwQosHistoryPolicy::Unknown => {
                        return Err(anyhow!("History set to unknown."));
                    }
                };

                let liveliness_dur =
                    std::time::Duration::from_secs(q.liveliness_lease_duration.sec)
                        + std::time::Duration::from_nanos(q.liveliness_lease_duration.nsec);
                let liveliness_dur = ros2::Duration::from_std(liveliness_dur);
                let liveliness = match RmwQosLivelinessPolicy::from_str(&q.liveliness)? {
                    RmwQosLivelinessPolicy::SystemDefault => {
                        ros2::policy::Liveliness::ManualByParticipant {
                            lease_duration: liveliness_dur,
                        }
                    }
                    RmwQosLivelinessPolicy::Automatic => ros2::policy::Liveliness::Automatic {
                        lease_duration: liveliness_dur,
                    },
                    RmwQosLivelinessPolicy::ManualByNode => {
                        ros2::policy::Liveliness::ManualByTopic {
                            lease_duration: liveliness_dur,
                        }
                    }
                    RmwQosLivelinessPolicy::ManualByTopic => {
                        ros2::policy::Liveliness::ManualByTopic {
                            lease_duration: liveliness_dur,
                        }
                    }
                    RmwQosLivelinessPolicy::Unknown => {
                        return Err(anyhow!("Liveliness set to unknown."));
                    }
                    RmwQosLivelinessPolicy::BestAvailable => todo!(),
                };
                ros2::QosPolicyBuilder::new()
                    .durability(durability)
                    .deadline(deadline)
                    .history(history)
                    .lifespan(lifespan)
                    .reliability(reliability)
                    .ownership(ros2::policy::Ownership::Shared)
                    .liveliness(liveliness)
                    .build()
            }
        })
    }
}

impl Bagfile {
    pub fn next(&mut self, kind: &PlayKindUnitedPass3) -> anyhow::Result<Vec<(u64, BagMsg)>> {
        match self.buffer.as_ref() {
            Some(buffer) => {
                let stream = mcap::MessageStream::new(buffer)?;
                let metadata = self.metadata.as_ref().expect("metadata set with buffer");
                match kind {
                    PlayKindUnitedPass3::SensorCount {
                        sensors,
                        count,
                        trigger: _,
                        play_mode: _,
                    } => collect_until(
                        stream,
                        &mut self.cursor,
                        metadata,
                        sensors,
                        None,
                        Some(count),
                    ),
                    PlayKindUnitedPass3::UntilSensorCount {
                        sending,
                        until_sensors,
                        until_count,
                        trigger: _,
                        play_mode: _,
                    } => collect_until(
                        stream,
                        &mut self.cursor,
                        metadata,
                        sending,
                        Some((until_sensors, until_count)),
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
    use rlc::PlayMode;

    use super::*;

    #[test]
    fn bag_read_simple() {
        let path = "../dlg_cut";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok());

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
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

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRelativeMs(50),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRangeMs(rlc::AbsTimeRange::Closed((50, 100))),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.len(), 1);

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
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
