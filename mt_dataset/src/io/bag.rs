use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use serde::{Deserialize, Serialize};
use walkdir::WalkDir;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BagSource {
    pub root: PathBuf,
    pub mcap: Option<PathBuf>,
    pub has_db3: bool,
    pub attachments: Vec<PathBuf>,
    pub original_bytes: u64,
}

pub fn discover_bag(path: &Path) -> Result<BagSource> {
    if !path.exists() {
        return Err(anyhow!("source path does not exist: {}", path.display()));
    }

    if path.is_file() {
        let meta = fs::metadata(path)?;
        let ext = path.extension().and_then(|e| e.to_str());
        return Ok(BagSource {
            root: path.to_path_buf(),
            mcap: (ext == Some("mcap")).then(|| path.to_path_buf()),
            has_db3: ext == Some("db3"),
            attachments: Vec::new(),
            original_bytes: meta.len(),
        });
    }

    let mut mcap = None;
    let mut attachments = Vec::new();
    let mut total: u64 = 0;
    let mut saw_ros2_db3 = false;

    for entry in WalkDir::new(path) {
        let entry = entry?;
        let p = entry.path();
        if p.is_dir() {
            continue;
        }

        let meta = fs::metadata(p)?;
        total += meta.len();

        let ext = p.extension().and_then(|v| v.to_str()).unwrap_or_default();
        match ext {
            "mcap" => {
                if mcap.is_none() {
                    mcap = Some(p.to_path_buf());
                }
            }
            "db3" => {
                saw_ros2_db3 = true;
                attachments.push(p.to_path_buf());
            }
            _ => attachments.push(p.to_path_buf()),
        }
    }

    Ok(BagSource {
        root: path.to_path_buf(),
        mcap,
        has_db3: saw_ros2_db3,
        attachments,
        original_bytes: total,
    })
}

pub fn has_direct_mcap(path: &Path) -> Result<bool> {
    if !path.exists() || !path.is_dir() {
        return Ok(false);
    }
    for entry in fs::read_dir(path).with_context(|| format!("reading {}", path.display()))? {
        let entry = entry?;
        let p = entry.path();
        if p.is_file() && p.extension().and_then(|e| e.to_str()) == Some("mcap") {
            return Ok(true);
        }
    }
    Ok(false)
}

pub fn has_direct_db3(path: &Path) -> Result<bool> {
    if !path.exists() || !path.is_dir() {
        return Ok(false);
    }
    for entry in fs::read_dir(path).with_context(|| format!("reading {}", path.display()))? {
        let entry = entry?;
        let p = entry.path();
        if p.is_file() && p.extension().and_then(|e| e.to_str()) == Some("db3") {
            return Ok(true);
        }
    }
    Ok(false)
}
