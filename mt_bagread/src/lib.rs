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
use mt_net::{BagMsg, Odometry, Qos, QosProfile, SensorTypeMapped};
pub use ros_pointcloud2::PointCloud2Msg;
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{Imu, PointCloud2};
use serde::{Deserialize, Serialize};
use serde::{Deserializer, de};
use std::fmt;

pub mod qos;
pub use mt_net::Qos::*;

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
    last_iter_time: Option<SystemTime>,
    start_time: Option<SystemTime>,
    metadata: Option<Metadata>,
    reader: Option<IndexedReader>,
    read_buffer: Vec<u8>,
    summary: Summary,
    file: Option<std::fs::File>,
}

impl fmt::Debug for Bagfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bagfile")
            .field("bagfile_name", &self.bagfile_name)
            .field("last_iter_time", &self.last_iter_time)
            .field("start_time", &self.start_time)
            .field("metadata", &self.metadata)
            // omitting reader
            .field(
                "read_buffer",
                &format!("[{} bytes]", self.read_buffer.len()),
            )
            .field("summary", &self.summary)
            // omitting "file",
            .finish()
    }
}

#[derive(Clone, Debug)]
pub struct BagReadResult {
    pub messages: Vec<(u64, BagMsg)>,
    pub end_of_bag: bool,
}

fn collect_until<T>(
    reader: &mut IndexedReader,
    file: &mut T,
    buffer: &mut Vec<u8>,
    last_iter_time: &mut Option<SystemTime>,
    start_time: &mut Option<SystemTime>,
    summary: &Summary,
    metadata: &Metadata,
    send_sensor: &Vec<AnySensor>,
    until_sensor: Option<(&Vec<AnySensor>, &PlayCount)>,
    until: Option<&PlayCount>,
) -> anyhow::Result<BagReadResult>
// TODO should be an iterator to remove warning for a ros2 bag play implementation
where
    T: Seek + Read,
{
    if start_time.is_none() {
        start_time.replace(nanos_to_system_time(
            summary.stats.as_ref().unwrap().message_start_time,
        ));
    }
    let start_time = start_time.unwrap();

    // reader
    let mut rel_since_begin = 0;
    let mut until_sensor_counter = 0;
    let mut msgs = Vec::new();
    let mut reached_end_naturally = true;

    // skipping if needed. if the request wants some data from earlier, redo the indexreader with the respective filters
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

    // reading messages
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
                    // just for completeness. can be handled at the end to avoid index reader init
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
                    for us in *until_sensors {
                        let pass = match &us.id {
                            SensorIdentification::Topic(topic) => topic.as_str() == &channel.topic,
                            SensorIdentification::Type(t) => t.is(channel.topic.as_str()),
                            SensorIdentification::TopicAndType { topic, msg_type: _ } => {
                                topic.as_str() == &channel.topic
                            }
                        };

                        if pass {
                            until_sensor_counter += 1;
                            match pc {
                                PlayCount::Amount(uc) => {
                                    if until_sensor_counter >= *uc {
                                        breaked_until = true;
                                        reached_end_naturally = false;
                                        break;
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

                    if breaked_until {
                        break;
                    }
                }

                let topic_meta = metadata
                    .get_topic_meta(&channel.topic)
                    .ok_or(anyhow!("Topic does not exist."))?;

                let mut send_type = SensorType::Any;
                let mut pass = false;
                for sensor in send_sensor.iter() {
                    let (n_pass, n_send_type) = match &sensor.id {
                        SensorIdentification::Topic(item) => {
                            (item.as_str() == &channel.topic, SensorType::Any)
                        }
                        SensorIdentification::Type(t) => (t.is(channel.topic.as_str()), *t),
                        SensorIdentification::TopicAndType { topic, msg_type: t } => {
                            (topic.as_str() == &channel.topic, *t)
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
                            let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
                                cdr::deserialize(data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            let data: PointCloud2 = unsafe { std::mem::transmute(dec) };

                            SensorTypeMapped::Lidar(data)
                        }
                        SensorType::Imu => {
                            let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::Imu =
                                cdr::deserialize(data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            let data: Imu = unsafe { std::mem::transmute(dec) };

                            SensorTypeMapped::Imu(data)
                        }
                        SensorType::Mixed => match topic_meta.topic_type.as_str() {
                            POINTCLOUD_ROS2_TYPE => {
                                let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
                                                        cdr::deserialize(data)
                                                            .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                                let data: PointCloud2 = unsafe { std::mem::transmute(dec) };

                                SensorTypeMapped::Lidar(data)
                            }
                            IMU_ROS2_TYPE => {
                                let dec: ros2_interfaces_jazzy_serde::sensor_msgs::msg::Imu =
                                    cdr::deserialize(data)
                                        .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                                let data: Imu = unsafe { std::mem::transmute(dec) };

                                SensorTypeMapped::Imu(data)
                            }
                            ODOM_ROS2_TYPE => {
                                let dec: ros2_interfaces_jazzy_serde::nav_msgs::msg::Odometry =
                                    cdr::deserialize(data)
                                        .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                                let data: Odometry = unsafe { std::mem::transmute(dec) };

                                SensorTypeMapped::Odometry(data)
                            }
                            _ => SensorTypeMapped::Any(data.to_vec()),
                        },
                        SensorType::Any => SensorTypeMapped::Any(data.to_vec()),
                        SensorType::Odom => {
                            let dec: ros2_interfaces_jazzy_serde::nav_msgs::msg::Odometry =
                                cdr::deserialize(data)
                                    .map_err(|e| anyhow!("Error decoding CDR: {e}"))?;
                            let data: Odometry = unsafe { std::mem::transmute(dec) };

                            SensorTypeMapped::Odometry(data)
                        }
                    };
                    let qos = topic_meta
                        .offered_qos_profiles
                        .last()
                        .map(|last| Qos::Custom(last.clone()));
                    let enc = BagMsg {
                        topic: topic_meta.name.clone(),
                        msg_type: topic_meta.topic_type.clone(),
                        data,
                        qos,
                    };
                    msgs.push((header.log_time, enc));
                }
                if len_before != msgs.len() {
                    rel_since_begin += 1;
                    if let Some(pc) = until {
                        if let PlayCount::Amount(count) = pc {
                            if rel_since_begin >= *count {
                                reached_end_naturally = false;
                                break;
                            }
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

impl Bagfile {
    pub fn next(&mut self, kind: &PlayKindUnitedPass3) -> anyhow::Result<BagReadResult> {
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
            let f = Utf8PathBuf::from_path_buf(mcap_path).unwrap();
            let mut file = fs::File::open(f).context("Couldn't open MCAP file")?;
            self.metadata = Some(bag_info);

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
            let reader = mcap::sans_io::indexed_reader::IndexedReader::new(&summary)
                .expect("could not construct reader");
            self.reader = Some(reader);
            self.read_buffer.resize(1024, 0);
            self.file = Some(file);
            self.summary = summary;
            self.start_time = None;
        }

        self.last_iter_time = None;
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
        // Verify that QoS profiles contain all expected fields with proper values
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
        assert_eq!(v5_qos.avoid_ros_namespace_conventions, false);

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
        assert_eq!(v9_qos.avoid_ros_namespace_conventions, false);

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
    #[ignore] // Requires actual bag file to be present
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
    #[ignore] // Requires actual bag file to be present
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
}
