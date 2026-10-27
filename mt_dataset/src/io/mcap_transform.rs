use std::borrow::Cow;
use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom};
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, anyhow};
use cloudini::ros::{CompressedPointCloud2, CompressionConfig};
use mcap::sans_io::{IndexedReadEvent, IndexedReader, SummaryReadEvent};
use mcap::{Compression, Message, Summary, WriteOptions, Writer};

use crate::io::transform_progress::{emit_count_progress, make_count_progress_bar};
use crate::progress::ProgressReporter;

ros_pointcloud2::impl_pointcloud2_for_ros2_interfaces_jazzy_serde!();

const POINTCLOUD2_SCHEMA: &str = "sensor_msgs/msg/PointCloud2";
const CDR_ENCODING: &str = "cdr";
const MARINA_CODEC_KEY: &str = "marina.pointcloud.codec";
const MARINA_CODEC_VAL: &str = "cloudini";
const CHUNK_PROGRESS_EVERY: usize = 16;
const MESSAGE_PROGRESS_EVERY: usize = 200_000;

/// Reads the profile from the header record that opens every MCAP file.
fn read_profile_from_file(file: &mut File) -> Result<String> {
    file.seek(SeekFrom::Start(0))
        .context("failed seeking to mcap header")?;
    let mut prefix = [0u8; mcap::MAGIC.len() + 1 + 8];
    file.read_exact(&mut prefix)
        .context("failed reading mcap header")?;
    let (magic, record) = prefix.split_at(mcap::MAGIC.len());
    if magic != mcap::MAGIC {
        return Err(anyhow!("not an mcap file: bad magic"));
    }
    let op = record[0];
    let length = u64::from_le_bytes(record[1..].try_into().expect("8 byte length"));
    let length = usize::try_from(length).context("mcap header length overflows usize")?;
    let mut body = vec![0u8; length];
    file.read_exact(&mut body)
        .context("failed reading mcap header body")?;
    match mcap::parse_record(op, &body).context("failed parsing mcap header")? {
        mcap::records::Record::Header(header) => Ok(header.profile),
        _ => Err(anyhow!("mcap file does not start with a header record")),
    }
}

fn read_summary_from_file(file: &mut File) -> Result<Summary> {
    let mut summary_reader = mcap::sans_io::summary_reader::SummaryReader::new();
    while let Some(event) = summary_reader.next_event() {
        match event.context("failed while reading mcap summary")? {
            SummaryReadEvent::ReadRequest(need) => {
                let written = file
                    .read(summary_reader.insert(need))
                    .context("failed reading mcap summary bytes")?;
                summary_reader.notify_read(written);
            }
            SummaryReadEvent::SeekRequest(to) => {
                let pos = file
                    .seek(to)
                    .context("failed seeking while reading mcap summary")?;
                summary_reader.notify_seeked(pos);
            }
        }
    }

    summary_reader
        .finish()
        .ok_or_else(|| anyhow!("MCAP summary missing. Indexed streaming requires a summary"))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum McapChunkCompression {
    None,
    #[default]
    Zstd,
    Lz4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PointCloudCompressionMode {
    Disabled,
    #[default]
    Lossy,
    Lossless,
}

#[derive(Debug, Clone, Copy)]
pub struct PushTransformOptions {
    pub pointcloud_mode: PointCloudCompressionMode,
    pub pointcloud_precision_m: f64,
    pub output_mcap_compression: McapChunkCompression,
}

impl Default for PushTransformOptions {
    fn default() -> Self {
        Self {
            pointcloud_mode: PointCloudCompressionMode::Lossy,
            pointcloud_precision_m: 0.001,
            output_mcap_compression: McapChunkCompression::Zstd,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PullTransformOptions {
    pub output_mcap_compression: McapChunkCompression,
}

impl Default for PullTransformOptions {
    fn default() -> Self {
        Self {
            output_mcap_compression: McapChunkCompression::Zstd,
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct TransformStats {
    pub pointcloud_messages: usize,
    pub total_messages: usize,
}

pub fn compress_mcap_for_push(input: &Path, output: &Path) -> Result<TransformStats> {
    let mut reporter = ProgressReporter::silent();
    compress_mcap_for_push_with_progress(
        input,
        output,
        PushTransformOptions::default(),
        &mut reporter,
    )
}

pub fn compress_mcap_for_push_with_progress(
    input: &Path,
    output: &Path,
    options: PushTransformOptions,
    progress: &mut ProgressReporter<'_>,
) -> Result<TransformStats> {
    let mut input_file = File::open(input)
        .with_context(|| format!("failed to open input mcap {}", input.display()))?;
    let profile = read_profile_from_file(&mut input_file)?;
    let summary = read_summary_from_file(&mut input_file)?;
    let mut reader =
        IndexedReader::new(&summary).context("failed constructing mcap indexed reader")?;
    let mut chunk_buffer = Vec::new();
    let total_chunks = summary.chunk_indexes.len();
    let expected_messages = summary.stats.as_ref().map(|s| s.message_count);
    let mut loaded_chunks = 0usize;
    let pb = make_count_progress_bar(total_chunks, "processing messages", "steps");
    let bar_visible = !pb.is_hidden();

    if !bar_visible {
        if let Some(total) = expected_messages {
            progress.emit(
                "pack",
                format!(
                    "reader initialized: {} chunk(s), ~{} message(s)",
                    total_chunks, total
                ),
            );
        } else {
            progress.emit(
                "pack",
                format!("reader initialized: {} chunk(s)", total_chunks),
            );
        }
    }

    let writer_file = File::create(output)
        .with_context(|| format!("failed to create output mcap {}", output.display()))?;
    let mut writer = Some(make_writer(
        BufWriter::new(writer_file),
        options.output_mcap_compression,
        &profile,
    )?);

    let mut stats = TransformStats::default();

    while let Some(event) = reader.next_event() {
        match event.context("failed reading indexed mcap events")? {
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                input_file
                    .seek(std::io::SeekFrom::Start(offset))
                    .with_context(|| format!("failed seeking to chunk at offset {}", offset))?;
                chunk_buffer.resize(length, 0);
                input_file.read_exact(&mut chunk_buffer).with_context(|| {
                    format!("failed reading chunk payload at offset {}", offset)
                })?;
                reader
                    .insert_chunk_record_data(offset, &chunk_buffer)
                    .context("failed inserting chunk data into mcap indexed reader")?;

                loaded_chunks += 1;
                if !pb.is_hidden() {
                    pb.inc(1);
                }
                if !bar_visible
                    && (loaded_chunks == 1
                        || loaded_chunks % CHUNK_PROGRESS_EVERY == 0
                        || loaded_chunks == total_chunks)
                {
                    emit_count_progress(progress, "pack", "processed", loaded_chunks, total_chunks);
                }
            }
            IndexedReadEvent::Message { header, data } => {
                let channel = summary
                    .channels
                    .get(&header.channel_id)
                    .ok_or_else(|| {
                        anyhow!(
                            "mcap message references unknown channel {}",
                            header.channel_id
                        )
                    })?
                    .clone();

                let msg = Message {
                    channel,
                    sequence: header.sequence,
                    log_time: header.log_time,
                    publish_time: header.publish_time,
                    data: Cow::Borrowed(data),
                };

                stats.total_messages += 1;
                if !bar_visible && stats.total_messages % MESSAGE_PROGRESS_EVERY == 0 {
                    if let Some(total) = expected_messages {
                        let pct = (stats.total_messages as f64 / total as f64) * 100.0;
                        progress.emit(
                            "pack",
                            format!(
                                "processed {} / {} message(s) ({:.1}%)",
                                stats.total_messages, total, pct
                            ),
                        );
                    } else {
                        progress.emit(
                            "pack",
                            format!("processed {} message(s)", stats.total_messages),
                        );
                    }
                }
                match options.pointcloud_mode {
                    PointCloudCompressionMode::Disabled => {
                        write_message_checked(&mut writer, &msg, "pack")?
                    }
                    PointCloudCompressionMode::Lossy | PointCloudCompressionMode::Lossless => {
                        if should_transform_channel(&msg) {
                            let transformed = compress_pointcloud_message(
                                msg,
                                options.pointcloud_mode,
                                options.pointcloud_precision_m,
                            )?;
                            write_message_checked(&mut writer, &transformed, "pack")?;
                            stats.pointcloud_messages += 1;
                        } else {
                            write_message_checked(&mut writer, &msg, "pack")?;
                        }
                    }
                }
            }
        }
    }

    finish_writer_checked(&mut writer, "pack")?;
    if !pb.is_hidden() {
        pb.finish_and_clear();
    }
    let mode = match options.pointcloud_mode {
        PointCloudCompressionMode::Disabled => "disabled",
        PointCloudCompressionMode::Lossy => "lossy",
        PointCloudCompressionMode::Lossless => "lossless",
    };
    if !bar_visible {
        progress.emit(
            "pack",
            format!(
                "reader finished: {} chunks loaded, {} of {} MCAP messages transformed as PointCloud2 (mode {}, precision {:.3} mm)",
                loaded_chunks,
                stats.pointcloud_messages,
                stats.total_messages,
                mode,
                options.pointcloud_precision_m * 1000.0
            ),
        );
    }
    Ok(stats)
}

pub fn has_cloudini_pointcloud_metadata(input: &Path) -> Result<bool> {
    let mut input_file = File::open(input)
        .with_context(|| format!("failed to open input mcap {}", input.display()))?;
    let summary = read_summary_from_file(&mut input_file)?;

    Ok(summary
        .channels
        .values()
        .any(|channel| channel.metadata.contains_key(MARINA_CODEC_KEY)))
}

pub fn decompress_mcap_after_pull(input: &Path, output: &Path) -> Result<TransformStats> {
    let mut reporter = ProgressReporter::silent();
    decompress_mcap_after_pull_with_progress(
        input,
        output,
        PullTransformOptions::default(),
        &mut reporter,
    )
}

pub fn decompress_mcap_after_pull_with_progress(
    input: &Path,
    output: &Path,
    options: PullTransformOptions,
    progress: &mut ProgressReporter<'_>,
) -> Result<TransformStats> {
    let mut input_file = File::open(input)
        .with_context(|| format!("failed to open input mcap {}", input.display()))?;
    let profile = read_profile_from_file(&mut input_file)?;
    let summary = read_summary_from_file(&mut input_file)?;
    let mut reader =
        IndexedReader::new(&summary).context("failed constructing mcap indexed reader")?;
    let mut chunk_buffer = Vec::new();
    let total_chunks = summary.chunk_indexes.len();
    let expected_messages = summary.stats.as_ref().map(|s| s.message_count);
    let mut loaded_chunks = 0usize;
    let pb = make_count_progress_bar(total_chunks, "restoring messages", "steps");
    let bar_visible = !pb.is_hidden();

    if !bar_visible {
        if let Some(total) = expected_messages {
            progress.emit(
                "unpack",
                format!(
                    "reader initialized: {} chunk(s), ~{} message(s)",
                    total_chunks, total
                ),
            );
        } else {
            progress.emit(
                "unpack",
                format!("reader initialized: {} chunk(s)", total_chunks),
            );
        }
    }

    let writer_file = File::create(output)
        .with_context(|| format!("failed to create output mcap {}", output.display()))?;
    let mut writer = Some(make_writer(
        BufWriter::new(writer_file),
        options.output_mcap_compression,
        &profile,
    )?);

    let mut stats = TransformStats::default();

    while let Some(event) = reader.next_event() {
        match event.context("failed reading indexed mcap events")? {
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                input_file
                    .seek(std::io::SeekFrom::Start(offset))
                    .with_context(|| format!("failed seeking to chunk at offset {}", offset))?;
                chunk_buffer.resize(length, 0);
                input_file.read_exact(&mut chunk_buffer).with_context(|| {
                    format!("failed reading chunk payload at offset {}", offset)
                })?;
                reader
                    .insert_chunk_record_data(offset, &chunk_buffer)
                    .context("failed inserting chunk data into mcap indexed reader")?;

                loaded_chunks += 1;
                if !pb.is_hidden() {
                    pb.inc(1);
                }
                if !bar_visible
                    && (loaded_chunks == 1
                        || loaded_chunks % CHUNK_PROGRESS_EVERY == 0
                        || loaded_chunks == total_chunks)
                {
                    emit_count_progress(
                        progress,
                        "unpack",
                        "processed",
                        loaded_chunks,
                        total_chunks,
                    );
                }
            }
            IndexedReadEvent::Message { header, data } => {
                let channel = summary
                    .channels
                    .get(&header.channel_id)
                    .ok_or_else(|| {
                        anyhow!(
                            "mcap message references unknown channel {}",
                            header.channel_id
                        )
                    })?
                    .clone();

                let msg = Message {
                    channel,
                    sequence: header.sequence,
                    log_time: header.log_time,
                    publish_time: header.publish_time,
                    data: Cow::Borrowed(data),
                };

                stats.total_messages += 1;
                if !bar_visible && stats.total_messages % MESSAGE_PROGRESS_EVERY == 0 {
                    if let Some(total) = expected_messages {
                        let pct = (stats.total_messages as f64 / total as f64) * 100.0;
                        progress.emit(
                            "unpack",
                            format!(
                                "processed {} / {} message(s) ({:.1}%)",
                                stats.total_messages, total, pct
                            ),
                        );
                    } else {
                        progress.emit(
                            "unpack",
                            format!("processed {} message(s)", stats.total_messages),
                        );
                    }
                }
                if is_cloudini_encoded_channel(&msg) {
                    let transformed = decompress_pointcloud_message(msg)?;
                    write_message_checked(&mut writer, &transformed, "unpack")?;
                    stats.pointcloud_messages += 1;
                } else {
                    write_message_checked(&mut writer, &msg, "unpack")?;
                }
            }
        }
    }

    finish_writer_checked(&mut writer, "unpack")?;
    if !pb.is_hidden() {
        pb.finish_and_clear();
    }
    if !bar_visible {
        progress.emit(
            "unpack",
            format!(
                "reader finished: {} chunks loaded, {} of {} MCAP messages restored as PointCloud2",
                loaded_chunks, stats.pointcloud_messages, stats.total_messages
            ),
        );
    }
    Ok(stats)
}

/// Compress a raw CDR-encoded PointCloud2 payload using cloudini.
/// Returns `(compressed_cdr_bytes, codec_value_string)`.
pub fn compress_cdr_pointcloud(
    data: &[u8],
    mode: PointCloudCompressionMode,
    precision_m: f64,
) -> Result<(Vec<u8>, String)> {
    let pointcloud: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
        cdr::deserialize(data).context("failed CDR-decoding PointCloud2")?;
    let cloud = impl_ros2_interfaces_jazzy_serde::to_pointcloud2_msg(pointcloud);
    let compression = match mode {
        PointCloudCompressionMode::Disabled => {
            unreachable!("disabled mode should not call compress_cdr_pointcloud")
        }
        PointCloudCompressionMode::Lossy => CompressionConfig::lossy_zstd(precision_m as f32),
        PointCloudCompressionMode::Lossless => CompressionConfig::lossless_zstd(),
    };
    let compressed = CompressedPointCloud2::compress(cloud, compression)
        .context("cloudini compression failed")?;
    let payload = cdr::serialize::<_, _, cdr::CdrLe>(&compressed, cdr::Infinite)
        .context("failed CDR-encoding compressed pointcloud")?;
    let codec_val = match mode {
        PointCloudCompressionMode::Disabled => unreachable!(),
        PointCloudCompressionMode::Lossy => format!("cloudini/lossy-zstd/{:.9}m", precision_m),
        PointCloudCompressionMode::Lossless => "cloudini/lossless-zstd".to_string(),
    };
    Ok((payload, codec_val))
}

/// Decompress cloudini-compressed CDR bytes back to standard PointCloud2 CDR bytes.
pub fn decompress_cdr_pointcloud(data: &[u8]) -> Result<Vec<u8>> {
    let compressed: CompressedPointCloud2 =
        cdr::deserialize(data).context("failed CDR-decoding compressed pointcloud")?;
    let restored = compressed
        .decompress()
        .context("cloudini decompression failed")?;
    let ros_pointcloud = impl_ros2_interfaces_jazzy_serde::from_pointcloud2_msg(restored);
    cdr::serialize::<_, _, cdr::CdrLe>(&ros_pointcloud, cdr::Infinite)
        .context("failed CDR-encoding restored PointCloud2")
}

/// Creates the output writer. `profile` is the input's, so a ROS 2 bag stays
/// recognizable as one after the transform.
fn make_writer(
    writer: BufWriter<File>,
    compression: McapChunkCompression,
    profile: &str,
) -> Result<Writer<BufWriter<File>>> {
    let mcap_compression = match compression {
        McapChunkCompression::None => None,
        McapChunkCompression::Zstd => Some(Compression::Zstd),
        McapChunkCompression::Lz4 => Some(Compression::Lz4),
    };
    let options = WriteOptions::new()
        .profile(profile)
        .compression(mcap_compression)
        .compression_threads(0);

    Writer::with_options(writer, options).context("failed creating mcap writer")
}

fn write_message_checked(
    writer: &mut Option<Writer<BufWriter<File>>>,
    msg: &Message<'_>,
    phase: &'static str,
) -> Result<()> {
    let mut w = writer
        .take()
        .ok_or_else(|| anyhow!("mcap writer missing during {}", phase))?;
    match w.write(msg) {
        Ok(()) => {
            *writer = Some(w);
            Ok(())
        }
        Err(err) => {
            std::mem::forget(w);
            Err(anyhow!(
                "failed writing {} mcap message: {} ({:?})",
                phase,
                err,
                err
            ))
        }
    }
}

fn finish_writer_checked(
    writer: &mut Option<Writer<BufWriter<File>>>,
    phase: &'static str,
) -> Result<()> {
    let mut w = writer
        .take()
        .ok_or_else(|| anyhow!("mcap writer missing during {} finish", phase))?;
    match w.finish() {
        Ok(_) => Ok(()),
        Err(err) => {
            std::mem::forget(w);
            Err(anyhow!(
                "failed finalizing {} mcap output: {} ({:?})",
                phase,
                err,
                err
            ))
        }
    }
}

fn should_transform_channel(msg: &Message<'_>) -> bool {
    msg.channel.message_encoding == CDR_ENCODING
        && msg
            .channel
            .schema
            .as_ref()
            .is_some_and(|s| s.name == POINTCLOUD2_SCHEMA)
}

fn is_cloudini_encoded_channel(msg: &Message<'_>) -> bool {
    msg.channel
        .metadata
        .get(MARINA_CODEC_KEY)
        .is_some_and(|v| v.starts_with(MARINA_CODEC_VAL))
}

fn compress_pointcloud_message<'a>(
    msg: Message<'a>,
    mode: PointCloudCompressionMode,
    precision_m: f64,
) -> Result<Message<'a>> {
    let pointcloud: ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 =
        cdr::deserialize(&msg.data)
            .context("failed to CDR-decode PointCloud2 while preparing push")?;

    let cloud = impl_ros2_interfaces_jazzy_serde::to_pointcloud2_msg(pointcloud);
    let compression = match mode {
        PointCloudCompressionMode::Disabled => {
            unreachable!("disabled mode should not reach compress_pointcloud_message")
        }
        PointCloudCompressionMode::Lossy => CompressionConfig::lossy_zstd(precision_m as f32),
        PointCloudCompressionMode::Lossless => CompressionConfig::lossless_zstd(),
    };
    let compressed = CompressedPointCloud2::compress(cloud, compression)
        .context("cloudini compression failed")?;

    let payload = cdr::serialize::<_, _, cdr::CdrLe>(&compressed, cdr::Infinite)
        .context("failed to CDR-encode compressed pointcloud")?;

    let mut metadata = msg.channel.metadata.clone();
    metadata.insert(
        MARINA_CODEC_KEY.to_string(),
        match mode {
            PointCloudCompressionMode::Disabled => "cloudini/disabled".to_string(),
            PointCloudCompressionMode::Lossy => {
                format!("cloudini/lossy-zstd/{:.9}m", precision_m)
            }
            PointCloudCompressionMode::Lossless => "cloudini/lossless-zstd".to_string(),
        },
    );

    let channel = Arc::new(mcap::Channel {
        id: msg.channel.id,
        topic: msg.channel.topic.clone(),
        schema: msg.channel.schema.clone(),
        message_encoding: msg.channel.message_encoding.clone(),
        metadata,
    });

    Ok(Message {
        channel,
        sequence: msg.sequence,
        log_time: msg.log_time,
        publish_time: msg.publish_time,
        data: Cow::Owned(payload),
    })
}

fn decompress_pointcloud_message<'a>(msg: Message<'a>) -> Result<Message<'a>> {
    let compressed: CompressedPointCloud2 = cdr::deserialize(&msg.data)
        .context("failed to CDR-decode compressed pointcloud while pulling")?;
    let restored = compressed
        .decompress()
        .context("cloudini decompression failed")?;

    let ros_pointcloud = impl_ros2_interfaces_jazzy_serde::from_pointcloud2_msg(restored);
    let payload = cdr::serialize::<_, _, cdr::CdrLe>(&ros_pointcloud, cdr::Infinite)
        .context("failed to CDR-encode restored PointCloud2")?;

    let mut metadata = msg.channel.metadata.clone();
    metadata.remove(MARINA_CODEC_KEY);

    let channel = Arc::new(mcap::Channel {
        id: msg.channel.id,
        topic: msg.channel.topic.clone(),
        schema: msg.channel.schema.clone(),
        message_encoding: msg.channel.message_encoding.clone(),
        metadata,
    });

    Ok(Message {
        channel,
        sequence: msg.sequence,
        log_time: msg.log_time,
        publish_time: msg.publish_time,
        data: Cow::Owned(payload),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_and_pull_keep_the_input_profile() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let input = dir.path().join("input.mcap");
        let packed = dir.path().join("packed.mcap");
        let restored = dir.path().join("restored.mcap");

        let mut writer = WriteOptions::new()
            .profile("ros2")
            .create(BufWriter::new(File::create(&input)?))?;
        let channel = writer.add_channel(0, "/chatter", CDR_ENCODING, &Default::default())?;
        writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: channel,
                sequence: 0,
                log_time: 1,
                publish_time: 1,
            },
            &[0, 1, 0, 0],
        )?;
        writer.finish()?;

        compress_mcap_for_push(&input, &packed)?;
        decompress_mcap_after_pull(&packed, &restored)?;

        assert_eq!(read_profile_from_file(&mut File::open(&packed)?)?, "ros2");
        assert_eq!(read_profile_from_file(&mut File::open(&restored)?)?, "ros2");
        Ok(())
    }
}
