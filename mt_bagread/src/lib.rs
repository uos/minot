use std::io::{Read, Seek};
use std::time::SystemTime;
use std::{fs, path::Path};

use anyhow::{Context, Result, anyhow};
use camino::Utf8PathBuf;
use mcap::Summary;
use mcap::records::{nanos_to_system_time, system_time_to_nanos};
use mcap::sans_io::{IndexedReadEvent, IndexedReader, IndexedReaderOptions, SummaryReadEvent};
use mt_mtc::{
    AbsTimeRange, AnySensor, IMU_ROS2_TYPE, ODOM_ROS2_TYPE, POINTCLOUD_ROS2_TYPE, PlayCount,
    PlayKindUnitedPass3, SensorIdentification, SensorType,
};
use mt_net::{BagMsg, Odometry, Qos, QosProfile, QosTime, SensorTypeMapped};
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{Imu, PointCloud2};
#[cfg(feature = "db3")]
use rusqlite::{Connection, OpenFlags, OptionalExtension};
use serde::{Deserialize, Serialize};
use serde::{Deserializer, de};
use std::fmt;

pub mod qos;
pub use mt_net::Qos::*;

#[cfg(feature = "db3")]
struct Db3State {
    conn: std::sync::Mutex<Connection>,
    cursor_ns: Option<u64>,
    start_ns: Option<u64>,
    topic_map: std::collections::HashMap<i64, (String, String, Vec<QosProfile>)>,
}

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

#[derive(Deserialize, Debug)]
pub struct Rosbag2BagfileInformation {
    pub version: i32,
    pub storage_identifier: String,
    pub duration: Nanoseconds,
    pub starting_time: NanosecondsSinceEpoch,
    pub message_count: i64,
    pub topics_with_message_count: Vec<TopicWithMessageCount>,
    pub compression_format: String,
    pub compression_mode: String,
    pub relative_file_paths: Vec<String>,
    pub files: Vec<File>,
    #[serde(default)]
    pub custom_data: Option<serde_yml::Value>,
    #[serde(default)]
    pub ros_distro: Option<String>,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct Nanoseconds {
    pub nanoseconds: u64,
}

#[derive(Deserialize, Debug, Clone, Copy)]
pub struct NanosecondsSinceEpoch {
    pub nanoseconds_since_epoch: u64,
}

#[derive(Deserialize, Debug)]
pub struct TopicWithMessageCount {
    pub topic_metadata: TopicMetadata,
    pub message_count: i64,
}

#[derive(Deserialize, Debug)]
pub struct TopicMetadata {
    pub name: String,
    #[serde(rename = "type")]
    pub topic_type: String,
    pub serialization_format: String,
    #[serde(deserialize_with = "deserialize_qos_profiles")]
    pub offered_qos_profiles: Vec<QosProfile>,
    #[serde(default)]
    pub type_description_hash: Option<String>,
}

#[derive(Deserialize, Debug)]
pub struct File {
    pub path: String,
    pub starting_time: NanosecondsSinceEpoch,
    pub duration: Nanoseconds,
    pub message_count: i64,
}

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
    let storage = &data.rosbag2_bagfile_information.storage_identifier;
    if storage != "mcap" && storage != "sqlite3" {
        return Err(anyhow!(
            "Only supporting mcap and sqlite3 storage. Please convert it."
        ));
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
        return Err(anyhow!("More than one file in a bagfile is not supported."));
    }

    Ok(())
}

fn default_qos_profiles() -> Vec<QosProfile> {
    vec![QosProfile {
        history: "keep_last".to_string(),
        depth: 10,
        reliability: "reliable".to_string(),
        durability: "volatile".to_string(),
        deadline: QosTime {
            sec: 9223372036,
            nsec: 854775807,
        },
        lifespan: QosTime {
            sec: 9223372036,
            nsec: 854775807,
        },
        liveliness: "automatic".to_string(),
        liveliness_lease_duration: QosTime {
            sec: 9223372036,
            nsec: 854775807,
        },
        avoid_ros_namespace_conventions: false,
    }]
}

fn metadata_from_summary(summary: &Summary) -> Metadata {
    let stats = summary.stats.as_ref();
    let topics_with_message_count: Vec<TopicWithMessageCount> = summary
        .channels
        .values()
        .map(|channel| {
            let topic_type = channel
                .schema
                .as_ref()
                .map(|s| s.name.clone())
                .unwrap_or_default();

            let offered_qos_profiles = channel
                .metadata
                .get("offered_qos_profiles")
                .and_then(|s| serde_yml::from_str(s).ok())
                .unwrap_or_else(default_qos_profiles);

            let message_count = stats
                .and_then(|s| s.channel_message_counts.get(&channel.id).copied())
                .unwrap_or(0) as i64;

            TopicWithMessageCount {
                topic_metadata: TopicMetadata {
                    name: channel.topic.clone(),
                    topic_type,
                    serialization_format: channel.message_encoding.clone(),
                    offered_qos_profiles,
                    type_description_hash: None,
                },
                message_count,
            }
        })
        .collect();

    let (start_ns, duration_ns, total_count) = if let Some(s) = stats {
        (
            s.message_start_time,
            s.message_end_time.saturating_sub(s.message_start_time),
            s.message_count as i64,
        )
    } else {
        (0, 0, 0)
    };

    Metadata {
        rosbag2_bagfile_information: Rosbag2BagfileInformation {
            version: 0,
            storage_identifier: "mcap".to_string(),
            duration: Nanoseconds {
                nanoseconds: duration_ns,
            },
            starting_time: NanosecondsSinceEpoch {
                nanoseconds_since_epoch: start_ns,
            },
            message_count: total_count,
            topics_with_message_count,
            compression_format: String::new(),
            compression_mode: String::new(),
            relative_file_paths: vec![],
            files: vec![],
            custom_data: None,
            ros_distro: None,
        },
    }
}

#[derive(Default)]
pub struct Bagfile {
    bagfile_name: Option<std::path::PathBuf>,
    last_iter_time: Option<SystemTime>,
    start_time: Option<SystemTime>,
    metadata: Option<Metadata>,
    reader: Option<IndexedReader>,
    read_buffer: Vec<u8>,
    summary: Summary,
    file: Option<std::fs::File>,
    #[cfg(feature = "db3")]
    db3: Option<Db3State>,
}

impl fmt::Debug for Bagfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut d = f.debug_struct("Bagfile");
        d.field("bagfile_name", &self.bagfile_name)
            .field("last_iter_time", &self.last_iter_time)
            .field("start_time", &self.start_time)
            .field("metadata", &self.metadata)
            // omitting reader
            .field(
                "read_buffer",
                &format!("[{} bytes]", self.read_buffer.len()),
            )
            .field("summary", &self.summary);
        // omitting "file"
        #[cfg(feature = "db3")]
        d.field("db3", &self.db3.as_ref().map(|_| "<db3>"));
        d.finish()
    }
}

#[derive(Clone, Debug)]
pub struct BagReadResult {
    pub messages: Vec<(u64, BagMsg)>,
    pub end_of_bag: bool,
}

fn match_send_sensor(send_sensor: &[AnySensor], topic_name: &str) -> (bool, SensorType) {
    let mut send_type = SensorType::Any;
    let mut pass = false;
    for sensor in send_sensor.iter() {
        let (n_pass, n_send_type) = match &sensor.id {
            SensorIdentification::Topic(item) => (item.as_str() == topic_name, SensorType::Any),
            SensorIdentification::Type(t) => (t.is(topic_name), *t),
            SensorIdentification::TopicAndType { topic, msg_type: t } => {
                (topic.as_str() == topic_name, *t)
            }
        };
        send_type = n_send_type;
        pass = n_pass;
        if n_pass {
            break;
        }
    }
    (pass, send_type)
}

fn deserialize_message(
    data: &[u8],
    send_type: SensorType,
    topic_type: &str,
) -> anyhow::Result<SensorTypeMapped> {
    match send_type {
        SensorType::Lidar => {
            let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
                cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
            let d: PointCloud2 = unsafe { std::mem::transmute(dec) };
            Ok(SensorTypeMapped::Lidar(d))
        }
        SensorType::Imu => {
            let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::Imu =
                cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
            let d: Imu = unsafe { std::mem::transmute(dec) };
            Ok(SensorTypeMapped::Imu(d))
        }
        SensorType::Mixed => match topic_type {
            POINTCLOUD_ROS2_TYPE => {
                let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
                    cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                let d: PointCloud2 = unsafe { std::mem::transmute(dec) };
                Ok(SensorTypeMapped::Lidar(d))
            }
            IMU_ROS2_TYPE => {
                let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::Imu =
                    cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                let d: Imu = unsafe { std::mem::transmute(dec) };
                Ok(SensorTypeMapped::Imu(d))
            }
            ODOM_ROS2_TYPE => {
                let dec: ros2_interfaces_jazzy_serde::nav_msgs::msg::Odometry =
                    cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                let d: Odometry = unsafe { std::mem::transmute(dec) };
                Ok(SensorTypeMapped::Odometry(d))
            }
            _ => Ok(SensorTypeMapped::Any(data.to_vec())),
        },
        SensorType::Any => Ok(SensorTypeMapped::Any(data.to_vec())),
        SensorType::Odom => {
            let dec: ros2_interfaces_jazzy_serde::nav_msgs::msg::Odometry =
                cdr::deserialize(data).map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
            let d: Odometry = unsafe { std::mem::transmute(dec) };
            Ok(SensorTypeMapped::Odometry(d))
        }
    }
}

fn check_until_sensor(
    until_sensors: &[AnySensor],
    topic_name: &str,
    counter: &mut u64,
    play_count: &PlayCount,
) -> anyhow::Result<bool> {
    for us in until_sensors {
        let pass = match &us.id {
            SensorIdentification::Topic(topic) => topic.as_str() == topic_name,
            SensorIdentification::Type(t) => t.is(topic_name),
            SensorIdentification::TopicAndType { topic, msg_type: _ } => {
                topic.as_str() == topic_name
            }
        };
        if pass {
            *counter += 1;
            match play_count {
                PlayCount::Amount(uc) => {
                    if *counter >= *uc {
                        return Ok(true);
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
    Ok(false)
}

#[allow(clippy::too_many_arguments)]
fn collect_until<T>(
    reader: &mut IndexedReader,
    file: &mut T,
    buffer: &mut Vec<u8>,
    last_iter_time: &mut Option<SystemTime>,
    start_time: &mut Option<SystemTime>,
    summary: &Summary,
    metadata: &Metadata,
    send_sensor: &[AnySensor],
    until_sensor: Option<(&Vec<AnySensor>, &PlayCount)>,
    until: Option<&PlayCount>,
) -> anyhow::Result<BagReadResult>
where
    T: Seek + Read,
{
    if start_time.is_none() {
        start_time.replace(nanos_to_system_time(
            summary.stats.as_ref().unwrap().message_start_time,
        ));
    }
    let start_time = start_time.unwrap();

    let mut rel_since_begin: u64 = 0;
    let mut until_sensor_counter: u64 = 0;
    let mut msgs = Vec::new();
    let mut reached_end_naturally = true;

    let lower = match until {
        Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, _))))
        | Some(PlayCount::TimeRangeMs(AbsTimeRange::UpperOpen(min))) => {
            let req_min = std::time::Duration::from_millis(*min);
            let abs_req_min = start_time + req_min;
            Some(abs_req_min)
        }
        Some(PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(_))) => Some(start_time),
        _ => None,
    };

    if let Some(lower_ts) = lower {
        let options =
            IndexedReaderOptions::new().log_time_on_or_after(system_time_to_nanos(&lower_ts));
        *reader = mcap::sans_io::indexed_reader::IndexedReader::new_with_options(summary, options)
            .expect("could not construct reader");
        *last_iter_time = Some(lower_ts);
    }

    let mut breaked_until = false;
    let mut time_iter_before = None;
    let span_start = last_iter_time.unwrap_or(start_time);
    while let Some(event) = reader.next_event() {
        match event? {
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                file.seek(std::io::SeekFrom::Start(offset))?;
                buffer.resize(length, 0);
                file.read_exact(buffer)?;
                reader.insert_chunk_record_data(offset, buffer)?;
            }
            IndexedReadEvent::Message { header, data } => {
                let channel = summary.channels.get(&header.channel_id).unwrap();
                let log_time = nanos_to_system_time(header.log_time);

                if time_iter_before.is_none() {
                    time_iter_before.replace(start_time);
                } else {
                    time_iter_before = *last_iter_time;
                }
                *last_iter_time = Some(nanos_to_system_time(header.log_time)); // advance cursor

                match until {
                    Some(PlayCount::TimeRelativeMs(rel_dur)) => {
                        let rel_dur = std::time::Duration::from_millis(*rel_dur);
                        let end_abs = span_start + rel_dur;
                        if log_time > end_abs {
                            breaked_until = true;
                            reached_end_naturally = false;
                            break;
                        }
                    }
                    Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, max)))) => {
                        let rel_dur = max - min;
                        let rel_dur = std::time::Duration::from_millis(rel_dur);
                        let end_abs = span_start + rel_dur;
                        if log_time > end_abs {
                            breaked_until = true;
                            reached_end_naturally = false;
                            break;
                        }
                    }
                    Some(PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(max))) => {
                        let max = std::time::Duration::from_millis(*max);
                        let end_abs = span_start + max;
                        if log_time > end_abs {
                            breaked_until = true;
                            reached_end_naturally = false;
                            break;
                        }
                    }
                    Some(PlayCount::Amount(count)) => {
                        if rel_since_begin >= *count {
                            breaked_until = true;
                            reached_end_naturally = false;
                            break;
                        }
                    }
                    _ => {}
                }

                if let Some((until_sensors, pc)) = &until_sensor {
                    if check_until_sensor(
                        until_sensors,
                        &channel.topic,
                        &mut until_sensor_counter,
                        pc,
                    )? {
                        breaked_until = true;
                        reached_end_naturally = false;
                        break;
                    }
                }

                let topic_meta = metadata
                    .get_topic_meta(&channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let (pass, send_type) = match_send_sensor(send_sensor, &channel.topic);

                let len_before = msgs.len();

                if pass {
                    let mapped = deserialize_message(data, send_type, &topic_meta.topic_type)?;
                    let qos = topic_meta
                        .offered_qos_profiles
                        .last()
                        .map(|last| Qos::Custom(last.clone()));
                    msgs.push((
                        header.log_time,
                        BagMsg {
                            topic: topic_meta.name.clone(),
                            msg_type: topic_meta.topic_type.clone(),
                            data: mapped,
                            qos,
                        },
                    ));
                }
                if len_before != msgs.len() {
                    rel_since_begin += 1;
                    if let Some(PlayCount::Amount(count)) = until {
                        if rel_since_begin >= *count {
                            reached_end_naturally = false;
                            break;
                        }
                    }
                }
            }
        }
    }

    // going back one message
    if breaked_until {
        let options = IndexedReaderOptions::new()
            .log_time_on_or_after(system_time_to_nanos(&time_iter_before.unwrap()));
        *reader = mcap::sans_io::indexed_reader::IndexedReader::new_with_options(summary, options)
            .expect("could not construct reader");
        *last_iter_time = time_iter_before;
    };

    Ok(BagReadResult {
        messages: msgs,
        end_of_bag: reached_end_naturally,
    })
}

#[cfg(feature = "db3")]
fn collect_until_db3(
    db3: &mut Db3State,
    send_sensor: &[AnySensor],
    until_sensor: Option<(&Vec<AnySensor>, &PlayCount)>,
    until: Option<&PlayCount>,
) -> anyhow::Result<BagReadResult> {
    let conn = db3.conn.lock().expect("db3 mutex poisoned");
    if db3.start_ns.is_none() {
        let min_ts: Option<i64> = conn
            .query_row("SELECT MIN(timestamp) FROM messages", [], |row| row.get(0))
            .optional()
            .context("failed to get start time from db3")?;
        db3.start_ns = Some(min_ts.unwrap_or(0) as u64);
    }
    let start_ns = db3.start_ns.unwrap();
    let span_start_ns = db3.cursor_ns.unwrap_or(start_ns);

    let lower_ns: Option<u64> = match until {
        Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, _))))
        | Some(PlayCount::TimeRangeMs(AbsTimeRange::UpperOpen(min))) => {
            Some(start_ns + *min * 1_000_000)
        }
        Some(PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(_))) => Some(start_ns),
        _ => None,
    };

    let query_from_ns = if let Some(lower) = lower_ns {
        db3.cursor_ns = Some(lower);
        lower
    } else {
        span_start_ns
    };

    let relevant_ids: Vec<i64> = db3
        .topic_map
        .iter()
        .filter(|(_, (name, _, _))| {
            let (send_pass, _) = match_send_sensor(send_sensor, name);
            let until_pass = until_sensor
                .map(|(us, _)| match_send_sensor(us, name).0)
                .unwrap_or(false);
            send_pass || until_pass
        })
        .map(|(id, _)| *id)
        .collect();

    let row_limit: Option<u64> = if until_sensor.is_none() {
        match until {
            Some(PlayCount::Amount(n)) => Some(*n),
            _ => None,
        }
    } else {
        None
    };

    let topic_filter = if relevant_ids.is_empty() {
        String::new()
    } else {
        let ids = relevant_ids
            .iter()
            .map(|id| id.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        format!(" AND m.topic_id IN ({ids})")
    };
    let limit_clause = row_limit.map_or(String::new(), |n| format!(" LIMIT {n}"));

    let sql = format!(
        "SELECT m.timestamp, m.data, m.topic_id \
         FROM messages m \
         WHERE m.timestamp >= ?1{topic_filter} ORDER BY m.timestamp{limit_clause}"
    );

    let topic_map = &db3.topic_map;
    let mut cursor_ns = db3.cursor_ns; // local cursor, applied to db3 after the loop

    let mut stmt = conn
        .prepare_cached(&sql)
        .context("failed preparing db3 message query")?;
    let mut rows_iter = stmt
        .query_map(rusqlite::params![query_from_ns as i64], |row| {
            Ok((
                row.get::<_, i64>(0)? as u64,
                row.get::<_, Vec<u8>>(1)?,
                row.get::<_, i64>(2)?,
            ))
        })
        .context("failed querying db3 messages")?;

    let mut rel_since_begin: u64 = 0;
    let mut until_sensor_counter: u64 = 0;
    let mut msgs: Vec<(u64, BagMsg)> = Vec::new();
    let mut reached_end_naturally = true;
    let mut breaked_until = false;
    let mut time_iter_before_ns: Option<u64> = None;

    for row_result in rows_iter.by_ref() {
        let (timestamp, data, topic_id) = row_result.context("failed reading db3 message row")?;

        let (topic_name, topic_type) = topic_map
            .get(&topic_id)
            .map(|(n, t, _)| (n.clone(), t.clone()))
            .ok_or_else(|| anyhow!("topic_id {topic_id} not in topic_map"))?;

        if time_iter_before_ns.is_none() {
            time_iter_before_ns = Some(span_start_ns);
        } else {
            time_iter_before_ns = cursor_ns;
        }
        cursor_ns = Some(timestamp);

        match until {
            Some(PlayCount::TimeRelativeMs(rel_dur)) => {
                let end_ns = span_start_ns + *rel_dur * 1_000_000;
                if timestamp > end_ns {
                    breaked_until = true;
                    reached_end_naturally = false;
                    break;
                }
            }
            Some(PlayCount::TimeRangeMs(AbsTimeRange::Closed((min, max)))) => {
                let rel_dur = max - min;
                let end_ns = span_start_ns + rel_dur * 1_000_000;
                if timestamp > end_ns {
                    breaked_until = true;
                    reached_end_naturally = false;
                    break;
                }
            }
            Some(PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(max))) => {
                let end_ns = span_start_ns + *max * 1_000_000;
                if timestamp > end_ns {
                    breaked_until = true;
                    reached_end_naturally = false;
                    break;
                }
            }
            Some(PlayCount::Amount(count)) => {
                if rel_since_begin >= *count {
                    breaked_until = true;
                    reached_end_naturally = false;
                    break;
                }
            }
            _ => {}
        }

        if let Some((until_sensors, pc)) = &until_sensor {
            if check_until_sensor(until_sensors, &topic_name, &mut until_sensor_counter, pc)? {
                breaked_until = true;
                reached_end_naturally = false;
                break;
            }
        }

        let (pass, send_type) = match_send_sensor(send_sensor, &topic_name);

        let len_before = msgs.len();
        if pass {
            let mapped = deserialize_message(&data, send_type, &topic_type)?;
            let qos = topic_map
                .get(&topic_id)
                .and_then(|(_, _, q)| q.last())
                .map(|last| Qos::Custom(last.clone()));
            msgs.push((
                timestamp,
                BagMsg {
                    topic: topic_name,
                    msg_type: topic_type,
                    data: mapped,
                    qos,
                },
            ));
        }
        if len_before != msgs.len() {
            rel_since_begin += 1;
            if let Some(PlayCount::Amount(count)) = until {
                if rel_since_begin >= *count {
                    reached_end_naturally = false;
                    break;
                }
            }
        }
    }

    if breaked_until {
        // Roll back to just before the message that triggered the break so the
        // next call replays from that point.
        db3.cursor_ns = time_iter_before_ns;
    } else {
        // Normal completion: advance the cursor one nanosecond past the last
        // message we processed so the next call doesn't re-read it.
        db3.cursor_ns = cursor_ns.map(|ns| ns.saturating_add(1));
    }

    Ok(BagReadResult {
        messages: msgs,
        end_of_bag: reached_end_naturally,
    })
}

impl Bagfile {
    pub fn next(&mut self, kind: &PlayKindUnitedPass3) -> anyhow::Result<BagReadResult> {
        #[cfg(feature = "db3")]
        if let Some(db3) = self.db3.as_mut() {
            return match kind {
                PlayKindUnitedPass3::SensorCount {
                    sensors,
                    count,
                    trigger: _,
                    play_mode: _,
                } => collect_until_db3(db3, sensors, None, Some(count)),
                PlayKindUnitedPass3::UntilSensorCount {
                    sending,
                    until_sensors,
                    until_count,
                    trigger: _,
                    play_mode: _,
                } => collect_until_db3(db3, sending, Some((until_sensors, until_count)), None),
            };
        }

        match self.reader.as_mut() {
            Some(reader) => {
                let metadata = self.metadata.as_ref().expect("metadata set with buffer");
                match kind {
                    PlayKindUnitedPass3::SensorCount {
                        sensors,
                        count,
                        trigger: _,
                        play_mode: _,
                    } => collect_until(
                        reader,
                        &mut self.file.as_mut().unwrap(),
                        &mut self.read_buffer,
                        &mut self.last_iter_time,
                        &mut self.start_time,
                        &self.summary,
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
                        reader,
                        &mut self.file.as_mut().unwrap(),
                        &mut self.read_buffer,
                        &mut self.last_iter_time,
                        &mut self.start_time,
                        &self.summary,
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

            // Support raw .db3 file without an accompanying metadata.yaml directory.
            let is_raw_db3 =
                path.is_file() && path.extension().map(|e| e == "db3").unwrap_or(false);
            if is_raw_db3 {
                #[cfg(not(feature = "db3"))]
                return Err(anyhow!(
                    "sqlite3/db3 bags require the 'db3' feature to be enabled."
                ));

                #[cfg(feature = "db3")]
                {
                    let conn = Connection::open_with_flags(
                        &path,
                        OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
                    )
                    .context("Couldn't open db3 file")?;
                    conn.execute_batch(
                        "PRAGMA cache_size = -65536;
                         PRAGMA temp_store = MEMORY;
                         PRAGMA mmap_size = 268435456;",
                    )
                    .context("failed setting db3 pragmas")?;
                    let mut topic_map = std::collections::HashMap::new();
                    {
                        let mut stmt = conn
                            .prepare("SELECT id, name, type, offered_qos_profiles FROM topics")
                            .context("failed to query topics table")?;
                        let rows = stmt
                            .query_map([], |row| {
                                Ok((
                                    row.get::<_, i64>(0)?,
                                    row.get::<_, String>(1)?,
                                    row.get::<_, String>(2)?,
                                    row.get::<_, String>(3)?,
                                ))
                            })
                            .context("failed to read topics")?;
                        for row in rows {
                            let (id, name, typ, qos_str) =
                                row.context("failed to read topic row")?;
                            let qos: Vec<QosProfile> =
                                serde_yml::from_str(&qos_str).unwrap_or_default();
                            topic_map.insert(id, (name, typ, qos));
                        }
                    }
                    self.metadata = None;
                    self.db3 = Some(Db3State {
                        conn: std::sync::Mutex::new(conn),
                        cursor_ns: None,
                        start_ns: None,
                        topic_map,
                    });
                    self.reader = None;
                    self.file = None;
                    self.start_time = None;
                }
            } else {
                let is_raw_mcap =
                    path.is_file() && path.extension().map(|e| e == "mcap").unwrap_or(false);

                // For directory bags, read optional metadata.
                let meta_contents: Option<String> = if !is_raw_mcap {
                    ["metadata.yaml", "metadata.yml"]
                        .iter()
                        .find_map(|name| fs::read_to_string(path.join(name)).ok())
                } else {
                    None
                };

                // Check for sqlite3 directory bag before opening any MCAP.
                let is_sqlite3_bag = meta_contents
                    .as_deref()
                    .and_then(|c| serde_yml::from_str::<Metadata>(c).ok())
                    .map(|m| m.rosbag2_bagfile_information.storage_identifier == "sqlite3")
                    .unwrap_or(false);

                if is_sqlite3_bag {
                    let bag_info: Metadata =
                        serde_yml::from_str(meta_contents.as_deref().unwrap())?;
                    validate_support(&bag_info)?;

                    #[cfg(not(feature = "db3"))]
                    return Err(anyhow!(
                        "sqlite3/db3 bags require the 'db3' feature to be enabled."
                    ));

                    #[cfg(feature = "db3")]
                    {
                        let db3_path = if let Some(file) =
                            bag_info.rosbag2_bagfile_information.files.first()
                        {
                            path.join(&file.path)
                        } else if let Some(rel) = bag_info
                            .rosbag2_bagfile_information
                            .relative_file_paths
                            .first()
                        {
                            path.join(rel)
                        } else {
                            return Err(anyhow!("No db3 file path found in bag metadata."));
                        };
                        let conn = Connection::open_with_flags(
                            &db3_path,
                            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_NO_MUTEX,
                        )
                        .context("Couldn't open db3 file")?;
                        conn.execute_batch(
                            "PRAGMA cache_size = -65536;
                             PRAGMA temp_store = MEMORY;
                             PRAGMA mmap_size = 268435456;",
                        )
                        .context("failed setting db3 pragmas")?;
                        let mut topic_map = std::collections::HashMap::new();
                        {
                            let mut stmt = conn
                                .prepare("SELECT id, name, type, offered_qos_profiles FROM topics")
                                .context("failed to query topics table")?;
                            let rows = stmt
                                .query_map([], |row| {
                                    Ok((
                                        row.get::<_, i64>(0)?,
                                        row.get::<_, String>(1)?,
                                        row.get::<_, String>(2)?,
                                        row.get::<_, String>(3)?,
                                    ))
                                })
                                .context("failed to read topics")?;
                            for row in rows {
                                let (id, name, typ, qos_str) =
                                    row.context("failed to read topic row")?;
                                let qos: Vec<QosProfile> =
                                    serde_yml::from_str(&qos_str).unwrap_or_default();
                                topic_map.insert(id, (name, typ, qos));
                            }
                        }
                        self.metadata = Some(bag_info);
                        self.db3 = Some(Db3State {
                            conn: std::sync::Mutex::new(conn),
                            cursor_ns: None,
                            start_ns: None,
                            topic_map,
                        });
                        self.reader = None;
                        self.file = None;
                        self.start_time = None;
                    }
                } else {
                    // All MCAP paths: raw .mcap file, directory+metadata, directory-no-metadata.
                    let (mcap_path, bag_info_opt): (std::path::PathBuf, Option<Metadata>) =
                        if is_raw_mcap {
                            (path.clone(), None)
                        } else if let Some(ref contents) = meta_contents {
                            let bag_info: Metadata = serde_yml::from_str(contents)?;
                            validate_support(&bag_info)?;
                            let mcap_file = bag_info
                                .rosbag2_bagfile_information
                                .files
                                .first()
                                .expect("Validated after deserialisation.");
                            let mcap_path = path.join(&mcap_file.path);
                            (mcap_path, Some(bag_info))
                        } else {
                            // No metadata — find the first .mcap in the directory.
                            let mcap_path = fs::read_dir(&path)
                                .with_context(|| format!("Cannot read directory {:?}", path))?
                                .filter_map(|e| e.ok())
                                .map(|e| e.path())
                                .find(|p| p.extension().is_some_and(|e| e == "mcap"))
                                .ok_or_else(|| {
                                    anyhow!("No .mcap or metadata.yaml found in {:?}", path)
                                })?;
                            (mcap_path, None)
                        };

                    let f = Utf8PathBuf::from_path_buf(mcap_path).unwrap();
                    let mut file = fs::File::open(f).context("Couldn't open MCAP file")?;

                    let summary = {
                        let mut reader = mcap::sans_io::summary_reader::SummaryReader::new();
                        while let Some(event) = reader.next_event() {
                            match event? {
                                SummaryReadEvent::ReadRequest(need) => {
                                    let written = file.read(reader.insert(need))?;
                                    reader.notify_read(written);
                                }
                                SummaryReadEvent::SeekRequest(to) => {
                                    reader.notify_seeked(file.seek(to)?);
                                }
                            }
                        }
                        reader.finish().unwrap()
                    };

                    self.metadata =
                        Some(bag_info_opt.unwrap_or_else(|| metadata_from_summary(&summary)));
                    let reader = mcap::sans_io::indexed_reader::IndexedReader::new(&summary)
                        .expect("could not construct reader");
                    self.reader = Some(reader);
                    self.read_buffer.resize(1024, 0);
                    self.file = Some(file);
                    self.summary = summary;
                    self.start_time = None;
                    #[cfg(feature = "db3")]
                    {
                        self.db3 = None;
                    }
                }
            } // end else { // not a raw .db3 file
        }

        self.last_iter_time = None;
        #[cfg(feature = "db3")]
        if let Some(db3) = self.db3.as_mut() {
            db3.cursor_ns = None;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use mt_mtc::PlayMode;

    use super::*;

    #[test]
    fn metadata_v5_parsing() {
        let metadata_contents = std::fs::read_to_string("test/metadata-v5.yml")
            .expect("Could not read test/metadata-v5.yml");
        let metadata: Metadata =
            serde_yml::from_str(&metadata_contents).expect("Failed to parse v5 metadata");

        // Verify basic fields
        assert_eq!(metadata.rosbag2_bagfile_information.version, 5);
        assert_eq!(
            metadata.rosbag2_bagfile_information.storage_identifier,
            "mcap"
        );
        assert_eq!(metadata.rosbag2_bagfile_information.message_count, 65178);

        // Verify topics parsed correctly
        assert_eq!(
            metadata
                .rosbag2_bagfile_information
                .topics_with_message_count
                .len(),
            4
        );

        // Check IMU topic QoS was parsed and normalized correctly
        let imu_topic = metadata
            .get_topic_meta("/imu/data")
            .expect("IMU topic not found");
        assert_eq!(imu_topic.name, "/imu/data");
        assert_eq!(imu_topic.topic_type, "sensor_msgs/msg/Imu");
        assert!(!imu_topic.offered_qos_profiles.is_empty());

        let qos = &imu_topic.offered_qos_profiles[0];
        assert_eq!(qos.history, "keep_last"); // 1 -> keep_last
        assert_eq!(qos.depth, 5);
        assert_eq!(qos.reliability, "reliable"); // 1 -> reliable
        assert_eq!(qos.durability, "volatile"); // 2 -> volatile
        assert_eq!(qos.liveliness, "automatic"); // 1 -> automatic

        // Check PointCloud2 topic
        let lidar_topic = metadata
            .get_topic_meta("/velodyne_points")
            .expect("Lidar topic not found");
        assert_eq!(lidar_topic.topic_type, "sensor_msgs/msg/PointCloud2");

        // Check tf_static has transient_local durability (1 -> transient_local)
        let tf_static = metadata
            .get_topic_meta("/tf_static")
            .expect("tf_static not found");
        let tf_qos = &tf_static.offered_qos_profiles[0];
        assert_eq!(tf_qos.durability, "transient_local"); // 1 -> transient_local
    }

    #[test]
    fn metadata_v9_parsing() {
        let metadata_contents = std::fs::read_to_string("test/metadata-v9.yml")
            .expect("Could not read test/metadata-v9.yml");
        let metadata: Metadata =
            serde_yml::from_str(&metadata_contents).expect("Failed to parse v9 metadata");

        // Verify basic fields
        assert_eq!(metadata.rosbag2_bagfile_information.version, 9);
        assert_eq!(
            metadata.rosbag2_bagfile_information.storage_identifier,
            "mcap"
        );
        assert_eq!(metadata.rosbag2_bagfile_information.message_count, 688909);

        // Verify optional v9 fields
        assert_eq!(
            metadata.rosbag2_bagfile_information.ros_distro,
            Some("jazzy".to_string())
        );

        // Verify topics parsed correctly
        assert!(
            metadata
                .rosbag2_bagfile_information
                .topics_with_message_count
                .len()
                >= 3
        );

        // Check PointCloud2 topic QoS was parsed correctly
        let lidar_topic = metadata
            .get_topic_meta("/velodyne_points")
            .expect("Lidar topic not found");
        assert_eq!(lidar_topic.name, "/velodyne_points");
        assert_eq!(lidar_topic.topic_type, "sensor_msgs/msg/PointCloud2");
        assert_eq!(lidar_topic.type_description_hash, Some("".to_string()));
        assert!(!lidar_topic.offered_qos_profiles.is_empty());

        let qos = &lidar_topic.offered_qos_profiles[0];
        assert_eq!(qos.history, "keep_last");
        assert_eq!(qos.depth, 10);
        assert_eq!(qos.reliability, "reliable");
        assert_eq!(qos.durability, "volatile");
        assert_eq!(qos.liveliness, "automatic");

        // Check tf_static has transient_local durability
        let tf_static = metadata
            .get_topic_meta("/tf_static")
            .expect("tf_static not found");
        let tf_qos = &tf_static.offered_qos_profiles[0];
        assert_eq!(tf_qos.durability, "transient_local");

        // Check multiple QoS profiles on /tf topic
        let tf_topic = metadata.get_topic_meta("/tf").expect("tf topic not found");
        assert_eq!(tf_topic.offered_qos_profiles.len(), 3);
    }

    #[test]
    fn qos_normalization_consistency() {
        // Parse both versions
        let v5_contents =
            std::fs::read_to_string("test/metadata-v5.yml").expect("Could not read v5 metadata");
        let v5_metadata: Metadata =
            serde_yml::from_str(&v5_contents).expect("Failed to parse v5 metadata");

        let v9_contents =
            std::fs::read_to_string("test/metadata-v9.yml").expect("Could not read v9 metadata");
        let v9_metadata: Metadata =
            serde_yml::from_str(&v9_contents).expect("Failed to parse v9 metadata");

        // Both should have normalized /velodyne_points with same QoS values
        let v5_lidar = v5_metadata
            .get_topic_meta("/velodyne_points")
            .expect("v5 lidar topic not found");
        let v9_lidar = v9_metadata
            .get_topic_meta("/velodyne_points")
            .expect("v9 lidar topic not found");

        let v5_qos = &v5_lidar.offered_qos_profiles[0];
        let v9_qos = &v9_lidar.offered_qos_profiles[0];

        // After normalization, both should have the same string values
        assert_eq!(v5_qos.history, v9_qos.history);
        assert_eq!(v5_qos.depth, v9_qos.depth);
        assert_eq!(v5_qos.reliability, v9_qos.reliability);
        assert_eq!(v5_qos.durability, v9_qos.durability);
        assert_eq!(v5_qos.liveliness, v9_qos.liveliness);
    }

    #[test]
    fn v5_offered_qos_profiles_stored_correctly() {
        let metadata_contents = std::fs::read_to_string("test/metadata-v5.yml")
            .expect("Could not read test/metadata-v5.yml");
        let metadata: Metadata =
            serde_yml::from_str(&metadata_contents).expect("Failed to parse v5 metadata");

        // Verify all topics have their QoS profiles stored
        for topic_with_count in &metadata
            .rosbag2_bagfile_information
            .topics_with_message_count
        {
            let topic_meta = &topic_with_count.topic_metadata;
            assert!(
                !topic_meta.offered_qos_profiles.is_empty(),
                "Topic {} should have QoS profiles",
                topic_meta.name
            );

            // Verify each QoS profile has normalized string values
            for qos in &topic_meta.offered_qos_profiles {
                // History should be normalized from numeric to string
                assert!(
                    qos.history == "keep_last" || qos.history == "keep_all",
                    "History should be normalized for topic {}: got {}",
                    topic_meta.name,
                    qos.history
                );

                // Reliability should be normalized
                assert!(
                    qos.reliability == "reliable" || qos.reliability == "best_effort",
                    "Reliability should be normalized for topic {}: got {}",
                    topic_meta.name,
                    qos.reliability
                );

                // Durability should be normalized
                assert!(
                    qos.durability == "volatile" || qos.durability == "transient_local",
                    "Durability should be normalized for topic {}: got {}",
                    topic_meta.name,
                    qos.durability
                );

                // Liveliness should be normalized
                assert!(
                    qos.liveliness == "automatic" || qos.liveliness == "manual_by_topic",
                    "Liveliness should be normalized for topic {}: got {}",
                    topic_meta.name,
                    qos.liveliness
                );
            }
        }

        // Test specific topics
        let imu = metadata.get_topic_meta("/imu/data").unwrap();
        assert_eq!(imu.offered_qos_profiles.len(), 1);
        assert_eq!(imu.offered_qos_profiles[0].depth, 5);

        let odom = metadata.get_topic_meta("/odom").unwrap();
        assert_eq!(odom.offered_qos_profiles.len(), 1);
        assert_eq!(odom.offered_qos_profiles[0].depth, 50);

        let tf_static = metadata.get_topic_meta("/tf_static").unwrap();
        assert_eq!(tf_static.offered_qos_profiles.len(), 1);
        assert_eq!(
            tf_static.offered_qos_profiles[0].durability,
            "transient_local"
        );

        let velodyne = metadata.get_topic_meta("/velodyne_points").unwrap();
        assert_eq!(velodyne.offered_qos_profiles.len(), 1);
        assert_eq!(velodyne.offered_qos_profiles[0].depth, 10);
    }

    #[test]
    fn v9_offered_qos_profiles_stored_correctly() {
        let metadata_contents = std::fs::read_to_string("test/metadata-v9.yml")
            .expect("Could not read test/metadata-v9.yml");
        let metadata: Metadata =
            serde_yml::from_str(&metadata_contents).expect("Failed to parse v9 metadata");

        // Verify all topics have their QoS profiles stored
        for topic_with_count in &metadata
            .rosbag2_bagfile_information
            .topics_with_message_count
        {
            let topic_meta = &topic_with_count.topic_metadata;
            assert!(
                !topic_meta.offered_qos_profiles.is_empty(),
                "Topic {} should have QoS profiles",
                topic_meta.name
            );

            // Verify type_description_hash field exists (v9 feature)
            assert!(
                topic_meta.type_description_hash.is_some(),
                "Topic {} should have type_description_hash field",
                topic_meta.name
            );
        }

        // Test /tf topic has 3 QoS profiles
        let tf = metadata.get_topic_meta("/tf").unwrap();
        assert_eq!(
            tf.offered_qos_profiles.len(),
            3,
            "/tf should have 3 QoS profiles"
        );
        for (i, qos) in tf.offered_qos_profiles.iter().enumerate() {
            assert_eq!(qos.history, "keep_last", "QoS profile {} history", i);
            assert_eq!(qos.depth, 100, "QoS profile {} depth", i);
            assert_eq!(qos.reliability, "reliable", "QoS profile {} reliability", i);
            assert_eq!(qos.durability, "volatile", "QoS profile {} durability", i);
            assert_eq!(qos.liveliness, "automatic", "QoS profile {} liveliness", i);
        }

        // Test other topics have single QoS profiles with correct values
        let velodyne = metadata.get_topic_meta("/velodyne_points").unwrap();
        assert_eq!(velodyne.offered_qos_profiles.len(), 1);
        assert_eq!(velodyne.offered_qos_profiles[0].depth, 10);
        assert_eq!(velodyne.offered_qos_profiles[0].durability, "volatile");

        let tf_static = metadata.get_topic_meta("/tf_static").unwrap();
        assert_eq!(tf_static.offered_qos_profiles.len(), 1);
        assert_eq!(tf_static.offered_qos_profiles[0].depth, 1);
        assert_eq!(
            tf_static.offered_qos_profiles[0].durability,
            "transient_local"
        );

        let imu = metadata.get_topic_meta("/imu/data").unwrap();
        assert_eq!(imu.offered_qos_profiles.len(), 1);
        assert_eq!(imu.offered_qos_profiles[0].depth, 5);
        assert_eq!(imu.offered_qos_profiles[0].durability, "volatile");

        let odom = metadata.get_topic_meta("/odom").unwrap();
        assert_eq!(odom.offered_qos_profiles.len(), 1);
        assert_eq!(odom.offered_qos_profiles[0].depth, 50);

        let joint_states = metadata.get_topic_meta("/joint_states").unwrap();
        assert_eq!(joint_states.offered_qos_profiles.len(), 1);
        assert_eq!(joint_states.offered_qos_profiles[0].depth, 10);
    }

    #[test]
    fn both_formats_parse_without_errors() {
        let v5_result = std::fs::read_to_string("test/metadata-v5.yml").and_then(|contents| {
            serde_yml::from_str::<Metadata>(&contents)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
        });
        assert!(
            v5_result.is_ok(),
            "v5 metadata should parse successfully: {:?}",
            v5_result.err()
        );

        let v9_result = std::fs::read_to_string("test/metadata-v9.yml").and_then(|contents| {
            serde_yml::from_str::<Metadata>(&contents)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
        });
        assert!(
            v9_result.is_ok(),
            "v9 metadata should parse successfully: {:?}",
            v9_result.err()
        );

        let v5_metadata = v5_result.unwrap();
        let v9_metadata = v9_result.unwrap();

        // Verify both have valid data
        assert!(v5_metadata.rosbag2_bagfile_information.message_count > 0);
        assert!(v9_metadata.rosbag2_bagfile_information.message_count > 0);
        assert!(
            !v5_metadata
                .rosbag2_bagfile_information
                .topics_with_message_count
                .is_empty()
        );
        assert!(
            !v9_metadata
                .rosbag2_bagfile_information
                .topics_with_message_count
                .is_empty()
        );
    }

    #[test]
    fn verify_qos_profiles_complete_data() {
        let v5_contents =
            std::fs::read_to_string("test/metadata-v5.yml").expect("Could not read v5 metadata");
        let v5_metadata: Metadata =
            serde_yml::from_str(&v5_contents).expect("Failed to parse v5 metadata");

        let v9_contents =
            std::fs::read_to_string("test/metadata-v9.yml").expect("Could not read v9 metadata");
        let v9_metadata: Metadata =
            serde_yml::from_str(&v9_contents).expect("Failed to parse v9 metadata");

        // Check V5 QoS profile completeness
        let v5_imu = v5_metadata.get_topic_meta("/imu/data").unwrap();
        let v5_qos = &v5_imu.offered_qos_profiles[0];

        assert_eq!(v5_qos.history, "keep_last");
        assert_eq!(v5_qos.depth, 5);
        assert_eq!(v5_qos.reliability, "reliable");
        assert_eq!(v5_qos.durability, "volatile");
        assert_eq!(v5_qos.deadline.sec, 9223372036);
        assert_eq!(v5_qos.deadline.nsec, 854775807);
        assert_eq!(v5_qos.lifespan.sec, 9223372036);
        assert_eq!(v5_qos.lifespan.nsec, 854775807);
        assert_eq!(v5_qos.liveliness, "automatic");
        assert_eq!(v5_qos.liveliness_lease_duration.sec, 9223372036);
        assert_eq!(v5_qos.liveliness_lease_duration.nsec, 854775807);
        assert!(!v5_qos.avoid_ros_namespace_conventions);

        // Check V9 QoS profile completeness
        let v9_imu = v9_metadata.get_topic_meta("/imu/data").unwrap();
        let v9_qos = &v9_imu.offered_qos_profiles[0];

        assert_eq!(v9_qos.history, "keep_last");
        assert_eq!(v9_qos.depth, 5);
        assert_eq!(v9_qos.reliability, "reliable");
        assert_eq!(v9_qos.durability, "volatile");
        assert_eq!(v9_qos.deadline.sec, 9223372036);
        assert_eq!(v9_qos.deadline.nsec, 854775807);
        assert_eq!(v9_qos.lifespan.sec, 9223372036);
        assert_eq!(v9_qos.lifespan.nsec, 854775807);
        assert_eq!(v9_qos.liveliness, "automatic");
        assert_eq!(v9_qos.liveliness_lease_duration.sec, 9223372036);
        assert_eq!(v9_qos.liveliness_lease_duration.nsec, 854775807);
        assert!(!v9_qos.avoid_ros_namespace_conventions);

        // Verify both have identical normalized values for the same topic
        assert_eq!(v5_qos.history, v9_qos.history);
        assert_eq!(v5_qos.depth, v9_qos.depth);
        assert_eq!(v5_qos.reliability, v9_qos.reliability);
        assert_eq!(v5_qos.durability, v9_qos.durability);
        assert_eq!(v5_qos.liveliness, v9_qos.liveliness);
        assert_eq!(v5_qos.deadline.sec, v9_qos.deadline.sec);
        assert_eq!(v5_qos.deadline.nsec, v9_qos.deadline.nsec);
    }

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
        assert_eq!(clouds.messages.len(), 1);
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
        assert_eq!(clouds.messages.len(), 1);

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRangeMs(mt_mtc::AbsTimeRange::Closed((30, 90))),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.messages.len(), 1);

        let clouds = bag.next(&PlayKindUnitedPass3::SensorCount {
            sensors: vec![AnySensor {
                name: "".to_owned(),
                id: SensorIdentification::Topic("/ouster/points".to_owned()),
                short: None,
            }],
            trigger: None,
            count: PlayCount::TimeRangeMs(mt_mtc::AbsTimeRange::Closed((0, 100))),
            play_mode: PlayMode::Fix,
        });

        assert!(clouds.is_ok());
        let clouds = clouds.unwrap();
        assert_eq!(clouds.messages.len(), 2);
    }

    #[test]
    fn bag_read_direct_mcap() {
        // Open a bare .mcap file without a metadata.yaml
        let path = "../dlg_cut/dlg_cut_0.mcap";
        let mut bag = Bagfile::default();
        let res = bag.reset(Some(path));
        assert!(res.is_ok(), "direct mcap open failed: {:?}", res.err());

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

        assert!(clouds.is_ok(), "read failed: {:?}", clouds.err());
        assert_eq!(clouds.unwrap().messages.len(), 1);
    }

    #[test]
    fn bag_read_dir_no_metadata() {
        // Directory with a .mcap but no metadata.yaml — should work via summary
        let tmp = std::env::temp_dir().join("dlg_cut_no_meta");
        std::fs::create_dir_all(&tmp).unwrap();
        // Symlink just the mcap, no metadata file
        let dst = tmp.join("dlg_cut_0.mcap");
        if !dst.exists() {
            std::os::unix::fs::symlink(
                std::fs::canonicalize("../dlg_cut/dlg_cut_0.mcap").unwrap(),
                &dst,
            )
            .unwrap();
        }

        let mut bag = Bagfile::default();
        let res = bag.reset(Some(&tmp));
        assert!(res.is_ok(), "dir-no-metadata open failed: {:?}", res.err());

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

        assert!(clouds.is_ok(), "read failed: {:?}", clouds.err());
        assert_eq!(clouds.unwrap().messages.len(), 1);
    }
}
