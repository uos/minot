use std::collections::HashMap;
use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use walkdir::WalkDir;

use crate::model::bag_ref::BagRef;
use crate::storage::config;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CacheEntry {
    pub bag: BagRef,
    pub local_dir: PathBuf,
    pub packed_bytes: u64,
    // original_bytes is intentionally absent — always derived from the local
    // directory via discover_bag so it stays accurate after a recording.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bundle_hash: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Catalog {
    pub entries: HashMap<String, CacheEntry>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct MirrorFile {
    pub path: String,
    pub size: u64,
    pub sha256: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MirrorReceivePlan {
    pub staging_dir: PathBuf,
    pub files: Vec<MirrorFile>,
    pub had_existing: bool,
}

fn catalog_path() -> Result<PathBuf> {
    Ok(config::config_dir()?.join("catalog.json"))
}

pub fn load_catalog() -> Result<Catalog> {
    let path = catalog_path()?;
    if !path.exists() {
        return Ok(Catalog::default());
    }

    let content =
        fs::read_to_string(&path).with_context(|| format!("failed reading {}", path.display()))?;
    let parsed = serde_json::from_str(&content)
        .with_context(|| format!("failed parsing {}", path.display()))?;
    Ok(parsed)
}

pub fn save_catalog(catalog: &Catalog) -> Result<()> {
    let path = catalog_path()?;
    let text = serde_json::to_string_pretty(catalog)?;
    fs::write(&path, text).with_context(|| format!("failed writing {}", path.display()))?;
    Ok(())
}

pub fn bag_cache_dir(bag: &BagRef) -> Result<PathBuf> {
    let root = config::cache_dir()?.join("bags").join(bag.cache_key());
    fs::create_dir_all(&root)?;
    Ok(root)
}

/// Prepare a private staging tree for a cache-to-cache mirror.
///
/// An existing ready tree is cloned with hard links where possible. This gives
/// the sender an incremental basis without exposing partial updates through the
/// catalog.
pub fn prepare_mirror_receive(bag: &BagRef) -> Result<MirrorReceivePlan> {
    let bag = bag.without_attachment();
    let cache_dir = bag_cache_dir(&bag)?;
    let ready_dir = cache_dir.join("ready");
    let had_existing = ready_dir.exists();
    let staging_dir = cache_dir.join(".mirror-incoming");
    if staging_dir.exists() {
        fs::remove_dir_all(&staging_dir)
            .with_context(|| format!("failed removing {}", staging_dir.display()))?;
    }
    if ready_dir.exists() {
        clone_tree(&ready_dir, &staging_dir)?;
    } else {
        fs::create_dir_all(&staging_dir)?;
    }
    Ok(MirrorReceivePlan {
        files: mirror_manifest(&staging_dir)?,
        staging_dir,
        had_existing,
    })
}

/// Atomically expose a staged mirrored dataset and register it in the catalog.
pub fn commit_mirror_receive(bag: &BagRef) -> Result<PathBuf> {
    let bag = bag.without_attachment();
    let cache_dir = bag_cache_dir(&bag)?;
    let ready_dir = cache_dir.join("ready");
    let staging_dir = cache_dir.join(".mirror-incoming");
    let backup_dir = cache_dir.join(".mirror-previous");
    if !staging_dir.is_dir() {
        anyhow::bail!("mirror staging directory is missing for '{}'", bag);
    }
    apply_native_mirror_manifest(&staging_dir)?;
    crate::io::bag::discover_bag(&staging_dir)
        .with_context(|| format!("invalid mirrored dataset '{}'", bag))?;

    if backup_dir.exists() {
        fs::remove_dir_all(&backup_dir)?;
    }
    if ready_dir.exists() {
        fs::rename(&ready_dir, &backup_dir)?;
    }
    if let Err(err) = fs::rename(&staging_dir, &ready_dir) {
        if backup_dir.exists() {
            let _ = fs::rename(&backup_dir, &ready_dir);
        }
        return Err(err).context("failed installing mirrored dataset");
    }

    let mut catalog = load_catalog()?;
    catalog.entries.insert(
        bag.to_string(),
        CacheEntry {
            bag,
            local_dir: ready_dir.clone(),
            packed_bytes: 0,
            bundle_hash: None,
        },
    );
    if let Err(err) = save_catalog(&catalog) {
        let _ = fs::remove_dir_all(&ready_dir);
        if backup_dir.exists() {
            let _ = fs::rename(&backup_dir, &ready_dir);
        }
        return Err(err).context("failed registering mirrored dataset");
    }
    if backup_dir.exists() {
        fs::remove_dir_all(&backup_dir)?;
    }
    Ok(ready_dir)
}

fn apply_native_mirror_manifest(staging_dir: &Path) -> Result<()> {
    let manifest_path = staging_dir.join(".marina-mirror-manifest.json");
    if !manifest_path.exists() {
        return Ok(());
    }
    let expected: Vec<MirrorFile> = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    fs::remove_file(&manifest_path)?;
    let expected_paths = expected
        .iter()
        .map(|file| file.path.as_str())
        .collect::<std::collections::HashSet<_>>();

    let mut directories = Vec::new();
    for entry in WalkDir::new(staging_dir).min_depth(1).follow_links(false) {
        let entry = entry?;
        let relative = entry
            .path()
            .strip_prefix(staging_dir)?
            .to_string_lossy()
            .replace('\\', "/");
        if entry.file_type().is_dir() {
            directories.push(entry.path().to_path_buf());
        } else if !expected_paths.contains(relative.as_str()) {
            fs::remove_file(entry.path())?;
        }
    }
    directories.sort_by_key(|path| std::cmp::Reverse(path.components().count()));
    for directory in directories {
        if fs::read_dir(&directory)?.next().is_none() {
            fs::remove_dir(&directory)?;
        }
    }

    let actual = mirror_manifest(staging_dir)?;
    if actual != expected {
        anyhow::bail!("native mirror verification failed: received files differ from sender");
    }
    Ok(())
}

pub fn mirror_manifest(root: &Path) -> Result<Vec<MirrorFile>> {
    let mut files = Vec::new();
    for entry in WalkDir::new(root).follow_links(false) {
        let entry = entry?;
        if entry.path() == root || entry.file_type().is_dir() {
            continue;
        }
        if entry.file_type().is_symlink() {
            anyhow::bail!(
                "symbolic links are not supported by native cache mirroring: {}",
                entry.path().display()
            );
        }
        let relative = entry.path().strip_prefix(root)?;
        let mut file = fs::File::open(entry.path())?;
        let mut hasher = Sha256::new();
        let mut buffer = [0u8; 1024 * 1024];
        let mut size = 0u64;
        loop {
            let read = file.read(&mut buffer)?;
            if read == 0 {
                break;
            }
            hasher.update(&buffer[..read]);
            size += read as u64;
        }
        files.push(MirrorFile {
            path: relative.to_string_lossy().replace('\\', "/"),
            size,
            sha256: hasher
                .finalize()
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect(),
        });
    }
    files.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(files)
}

fn clone_tree(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;
    for entry in WalkDir::new(src).min_depth(1).follow_links(false) {
        let entry = entry?;
        let relative = entry.path().strip_prefix(src)?;
        let target = dst.join(relative);
        if entry.file_type().is_dir() {
            fs::create_dir_all(&target)?;
        } else if entry.file_type().is_symlink() {
            anyhow::bail!(
                "symbolic links are not supported by native cache mirroring: {}",
                entry.path().display()
            );
        } else if let Err(link_err) = fs::hard_link(entry.path(), &target) {
            fs::copy(entry.path(), &target).with_context(|| {
                format!(
                    "failed cloning {} after hard-link error: {}",
                    entry.path().display(),
                    link_err
                )
            })?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirror_manifest_is_sorted_and_hashes_contents() -> Result<()> {
        let temp = tempfile::tempdir()?;
        fs::create_dir_all(temp.path().join("nested"))?;
        fs::write(temp.path().join("z.txt"), b"last")?;
        fs::write(temp.path().join("nested/a.txt"), b"first")?;

        let manifest = mirror_manifest(temp.path())?;
        assert_eq!(
            manifest.iter().map(|f| f.path.as_str()).collect::<Vec<_>>(),
            vec!["nested/a.txt", "z.txt"]
        );
        assert_eq!(manifest[0].size, 5);
        assert_eq!(
            manifest[0].sha256,
            "a7937b64b8caa58f03721bb6bacf5c78cb235febe0e70b1b84cd99541461a08e"
        );
        Ok(())
    }

    #[test]
    fn native_manifest_prunes_stale_files_and_verifies_result() -> Result<()> {
        let temp = tempfile::tempdir()?;
        fs::create_dir_all(temp.path().join("stale-dir"))?;
        fs::write(temp.path().join("keep.mcap"), b"dataset")?;
        fs::write(temp.path().join("stale-dir/old.txt"), b"old")?;
        let expected = vec![MirrorFile {
            path: "keep.mcap".to_string(),
            size: 7,
            sha256: "b277fd623676a525c29b9eb155afc8c9010681814ceafb2d7627f47b9a232576".to_string(),
        }];
        fs::write(
            temp.path().join(".marina-mirror-manifest.json"),
            serde_json::to_vec(&expected)?,
        )?;

        apply_native_mirror_manifest(temp.path())?;

        assert!(!temp.path().join("stale-dir").exists());
        assert_eq!(mirror_manifest(temp.path())?, expected);
        Ok(())
    }
}
