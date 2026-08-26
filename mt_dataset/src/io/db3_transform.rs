use std::path::Path;

use anyhow::{Context, Result};
use rusqlite::{Connection, params};

use crate::io::mcap_transform::{
    PointCloudCompressionMode, compress_cdr_pointcloud, decompress_cdr_pointcloud,
};
use crate::io::transform_progress::{emit_count_progress, make_count_progress_bar};
use crate::progress::ProgressReporter;

const MARINA_CODEC_KEY: &str = "marina.pointcloud.codec";
const POINTCLOUD2_TYPE: &str = "sensor_msgs/msg/PointCloud2";
const MESSAGE_PROGRESS_EVERY: usize = 25;

#[derive(Debug, Default, Clone, Copy)]
pub struct Db3TransformStats {
    pub pointcloud_messages: usize,
    pub total_messages: usize,
}

pub struct Db3TransformOptions {
    pub pointcloud_mode: PointCloudCompressionMode,
    pub pointcloud_precision_m: f64,
    pub vacuum_after_transform: bool,
}

pub fn has_marina_pointcloud_metadata(db3_path: &Path) -> Result<bool> {
    let conn = Connection::open(db3_path)
        .with_context(|| format!("failed to open db3 {}", db3_path.display()))?;

    let table_exists: bool = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='marina_metadata'",
            [],
            |row| row.get::<_, i64>(0),
        )
        .context("failed querying sqlite_master")?
        > 0;

    if !table_exists {
        return Ok(false);
    }

    let count: i64 = conn
        .query_row(
            "SELECT COUNT(*) FROM marina_metadata WHERE key = ?1",
            params![MARINA_CODEC_KEY],
            |row| row.get(0),
        )
        .context("failed querying marina_metadata")?;

    Ok(count > 0)
}

pub fn compress_db3_for_push(
    db3_path: &Path,
    options: &Db3TransformOptions,
    progress: &mut ProgressReporter<'_>,
) -> Result<Db3TransformStats> {
    let mut stats = Db3TransformStats::default();

    let conn = Connection::open(db3_path)
        .with_context(|| format!("failed to open db3 {}", db3_path.display()))?;

    conn.execute_batch(
        "PRAGMA cache_size = -65536;
         PRAGMA temp_store = MEMORY;",
    )
    .context("failed setting db3 pragmas")?;

    if !has_rosbag_db3_schema(&conn)? {
        progress.emit(
            "pack",
            "db3 file has an unsupported schema. Skipping db3 transform",
        );
        return Ok(stats);
    }

    let topic_ids = pointcloud_topic_ids(&conn)?;
    if topic_ids.is_empty() {
        progress.emit("pack", "no PointCloud2 topics found in db3");
        return Ok(stats);
    }

    stats.total_messages = conn
        .query_row("SELECT COUNT(*) FROM messages", [], |row| {
            row.get::<_, i64>(0)
        })
        .context("failed counting messages")? as usize;

    let placeholders = topic_ids.iter().map(|_| "?").collect::<Vec<_>>().join(",");
    let pc2_count: usize = conn
        .query_row(
            &format!("SELECT COUNT(*) FROM messages WHERE topic_id IN ({placeholders})"),
            rusqlite::params_from_iter(topic_ids.iter()),
            |row| row.get::<_, i64>(0),
        )
        .context("failed counting PointCloud2 messages")? as usize;
    stats.pointcloud_messages = pc2_count;

    let mode_str = match options.pointcloud_mode {
        PointCloudCompressionMode::Disabled => "disabled",
        PointCloudCompressionMode::Lossy => "lossy",
        PointCloudCompressionMode::Lossless => "lossless",
    };
    let pb = make_count_progress_bar(pc2_count, "processing messages", "steps");
    let bar_visible = !pb.is_hidden();
    if !bar_visible {
        progress.emit(
            "pack",
            format!(
                "compressing {} messages (mode: {}, precision: {:.3} mm)",
                pc2_count,
                mode_str,
                options.pointcloud_precision_m * 1000.0
            ),
        );
    }

    conn.execute_batch("BEGIN EXCLUSIVE")
        .context("failed beginning db3 transaction")?;

    let result = (|| -> Result<String> {
        // Stream (id, data) in one pass. Prepare the update statement once.
        let select_sql =
            format!("SELECT id, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY id");
        let mut read_stmt = conn
            .prepare(&select_sql)
            .context("failed preparing message select query")?;
        let mut write_stmt = conn
            .prepare_cached("UPDATE messages SET data = ?1 WHERE id = ?2")
            .context("failed preparing message update query")?;

        let mut rows = read_stmt
            .query_map(rusqlite::params_from_iter(topic_ids.iter()), |row| {
                Ok((row.get::<_, i64>(0)?, row.get::<_, Vec<u8>>(1)?))
            })
            .context("failed querying messages")?;

        let mut codec_val = String::new();
        let mut i: usize = 0;
        for row_result in rows.by_ref() {
            let (id, data) = row_result.with_context(|| "failed reading message row")?;

            let (compressed, cv) = compress_cdr_pointcloud(
                &data,
                options.pointcloud_mode,
                options.pointcloud_precision_m,
            )
            .with_context(|| format!("failed compressing PointCloud2 message id={id}"))?;
            codec_val = cv;

            write_stmt
                .execute(params![compressed, id])
                .with_context(|| format!("failed updating message id={id}"))?;

            pb.inc(1);
            i += 1;
            if !bar_visible && (i % MESSAGE_PROGRESS_EVERY == 0 || i == pc2_count) {
                emit_count_progress(progress, "pack", "processed", i, pc2_count);
            }
        }
        Ok(codec_val)
    })();

    match result {
        Ok(codec_val) => {
            conn.execute_batch(
                "CREATE TABLE IF NOT EXISTS marina_metadata \
                 (key TEXT PRIMARY KEY, value TEXT NOT NULL)",
            )
            .context("failed creating marina_metadata table")?;
            conn.execute(
                "INSERT OR REPLACE INTO marina_metadata VALUES (?1, ?2)",
                params![MARINA_CODEC_KEY, codec_val],
            )
            .context("failed inserting marina metadata")?;
            conn.execute_batch("COMMIT")
                .context("failed committing db3 transaction")?;
            pb.finish_and_clear();

            if options.vacuum_after_transform {
                if !bar_visible {
                    progress.emit("pack", "vacuuming db3 to reclaim freed pages");
                }
                if let Err(err) = conn.execute_batch("VACUUM") {
                    if !bar_visible {
                        progress.emit("pack", format!("db3 vacuum skipped: {err}"));
                    }
                }
            } else if !bar_visible {
                progress.emit("pack", "skipping db3 vacuum");
            }

            if !bar_visible {
                progress.emit(
                    "pack",
                    format!(
                        "db3 compression complete: {} PointCloud2 message(s) compressed",
                        pc2_count
                    ),
                );
            }
        }
        Err(e) => {
            let _ = conn.execute_batch("ROLLBACK");
            pb.abandon();
            return Err(e);
        }
    }

    Ok(stats)
}

pub fn decompress_db3_after_pull(
    db3_path: &Path,
    progress: &mut ProgressReporter<'_>,
) -> Result<Db3TransformStats> {
    let mut stats = Db3TransformStats::default();

    let conn = Connection::open(db3_path)
        .with_context(|| format!("failed to open db3 {}", db3_path.display()))?;

    conn.execute_batch(
        "PRAGMA cache_size = -65536;
         PRAGMA temp_store = MEMORY;",
    )
    .context("failed setting db3 pragmas")?;

    if !has_rosbag_db3_schema(&conn)? {
        return Ok(stats);
    }

    let topic_ids = pointcloud_topic_ids(&conn)?;
    if topic_ids.is_empty() {
        return Ok(stats);
    }

    stats.total_messages = conn
        .query_row("SELECT COUNT(*) FROM messages", [], |row| {
            row.get::<_, i64>(0)
        })
        .context("failed counting messages")? as usize;

    let placeholders = topic_ids.iter().map(|_| "?").collect::<Vec<_>>().join(",");
    let pc2_count: usize = conn
        .query_row(
            &format!("SELECT COUNT(*) FROM messages WHERE topic_id IN ({placeholders})"),
            rusqlite::params_from_iter(topic_ids.iter()),
            |row| row.get::<_, i64>(0),
        )
        .context("failed counting PointCloud2 messages")? as usize;
    stats.pointcloud_messages = pc2_count;

    let pb = make_count_progress_bar(pc2_count, "restoring messages", "steps");
    let bar_visible = !pb.is_hidden();
    if !bar_visible {
        progress.emit(
            "unpack",
            format!("restoring {} PointCloud2 message(s) in db3", pc2_count),
        );
    }

    conn.execute_batch("BEGIN EXCLUSIVE")
        .context("failed beginning db3 transaction")?;

    let result = (|| -> Result<()> {
        let select_sql =
            format!("SELECT id, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY id");
        let mut read_stmt = conn
            .prepare(&select_sql)
            .context("failed preparing message select query")?;
        let mut write_stmt = conn
            .prepare_cached("UPDATE messages SET data = ?1 WHERE id = ?2")
            .context("failed preparing message update query")?;

        let mut rows = read_stmt
            .query_map(rusqlite::params_from_iter(topic_ids.iter()), |row| {
                Ok((row.get::<_, i64>(0)?, row.get::<_, Vec<u8>>(1)?))
            })
            .context("failed querying messages")?;

        let mut i: usize = 0;
        for row_result in rows.by_ref() {
            let (id, data) = row_result.with_context(|| "failed reading message row")?;

            let restored = decompress_cdr_pointcloud(&data)
                .with_context(|| format!("failed decompressing PointCloud2 message id={id}"))?;

            write_stmt
                .execute(params![restored, id])
                .with_context(|| format!("failed updating message id={id}"))?;

            pb.inc(1);
            i += 1;
            if !bar_visible && (i % MESSAGE_PROGRESS_EVERY == 0 || i == pc2_count) {
                emit_count_progress(progress, "unpack", "processed", i, pc2_count);
            }
        }
        Ok(())
    })();

    match result {
        Ok(()) => {
            let _ = conn.execute(
                "DELETE FROM marina_metadata WHERE key = ?1",
                params![MARINA_CODEC_KEY],
            );
            conn.execute_batch("COMMIT")
                .context("failed committing db3 transaction")?;
            pb.finish_and_clear();
            if !bar_visible {
                progress.emit(
                    "unpack",
                    format!(
                        "db3 restore complete: {} PointCloud2 message(s) restored",
                        pc2_count
                    ),
                );
            }
        }
        Err(e) => {
            let _ = conn.execute_batch("ROLLBACK");
            pb.abandon();
            return Err(e);
        }
    }

    Ok(stats)
}

fn pointcloud_topic_ids(conn: &Connection) -> Result<Vec<i64>> {
    let mut stmt = conn
        .prepare("SELECT id FROM topics WHERE type = ?1")
        .context("failed preparing topics query")?;
    stmt.query_map(params![POINTCLOUD2_TYPE], |row| row.get(0))
        .context("failed querying topics")?
        .collect::<Result<Vec<i64>, _>>()
        .context("failed collecting topic ids")
}

fn has_rosbag_db3_schema(conn: &Connection) -> Result<bool> {
    let topics_exists: bool = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='topics'",
            [],
            |row| row.get::<_, i64>(0),
        )
        .context("failed checking topics table")?
        > 0;
    let messages_exists: bool = conn
        .query_row(
            "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='messages'",
            [],
            |row| row.get::<_, i64>(0),
        )
        .context("failed checking messages table")?
        > 0;
    Ok(topics_exists && messages_exists)
}
