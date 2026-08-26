use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use async_trait::async_trait;
use glob::Pattern;
use serde::{Deserialize, Serialize};
use walkdir::WalkDir;

use crate::model::bag_ref::BagRef;
use crate::registry::driver::{BagInfo, PushMeta, RegistryDriver, RemoteDescriptor};

#[derive(Debug, Clone)]
pub struct FolderRegistry {
    pub name: String,
    pub root: PathBuf,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct MetaFile {
    bag: BagRef,
    original_bytes: u64,
    packed_bytes: u64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    tags: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    bundle_hash: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pointcloud: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    mcap_compression: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pushed_at: Option<u64>,
}

#[derive(Debug, Clone, Serialize)]
struct HttpIndexEntry {
    bag: BagRef,
    original_bytes: u64,
    packed_bytes: u64,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    tags: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
struct HttpIndexFile {
    bags: Vec<HttpIndexEntry>,
}

impl FolderRegistry {
    pub fn from_uri(name: &str, uri: &str) -> Result<Self> {
        let raw = if let Some(rest) = uri.strip_prefix("folder://") {
            PathBuf::from(rest)
        } else if let Some(rest) = uri.strip_prefix("folder::") {
            PathBuf::from(rest)
        } else if let Some(rest) = uri.strip_prefix("directory://") {
            PathBuf::from(rest)
        } else if let Some(rest) = uri.strip_prefix("directory::") {
            PathBuf::from(rest)
        } else {
            PathBuf::from(uri)
        };
        let root = if raw.is_relative() {
            std::env::current_dir()
                .context("failed to determine current directory")?
                .join(&raw)
        } else {
            raw
        };
        fs::create_dir_all(&root)?;
        Ok(Self {
            name: name.to_string(),
            root,
        })
    }

    fn object_dir(&self, bag: &BagRef) -> PathBuf {
        self.root.join(bag.object_path())
    }

    fn meta_path(&self, bag: &BagRef) -> PathBuf {
        self.object_dir(bag).join("metadata.json")
    }

    fn data_path(&self, bag: &BagRef) -> PathBuf {
        self.object_dir(bag).join("bundle.marina.tar.gz")
    }

    pub fn source_bundle_path(&self, bag: &BagRef) -> PathBuf {
        self.data_path(bag)
    }

    pub fn finalize_existing_bundle(&self, bag: &BagRef, meta: &PushMeta) -> Result<()> {
        let target_dir = self.object_dir(bag);
        fs::create_dir_all(&target_dir)?;

        let meta_file = MetaFile {
            bag: bag.clone().without_attachment(),
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            tags: bag.tags.clone(),
            bundle_hash: Some(meta.bundle_hash.clone()),
            pointcloud: Some(meta.pointcloud.clone()),
            mcap_compression: Some(meta.mcap_compression.clone()),
            pushed_at: Some(meta.pushed_at),
        };
        fs::write(
            self.meta_path(bag),
            serde_json::to_string_pretty(&meta_file)?,
        )?;
        Ok(())
    }

    fn read_meta(&self, bag: &BagRef) -> Result<MetaFile> {
        let p = self.meta_path(bag);
        let text = fs::read_to_string(&p)
            .with_context(|| format!("failed to read metadata {}", p.display()))?;
        let meta: MetaFile = serde_json::from_str(&text)
            .with_context(|| format!("failed to parse metadata {}", p.display()))?;
        Ok(meta)
    }
}

#[async_trait]
impl RegistryDriver for FolderRegistry {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    async fn push(
        &self,
        _registry_name: &str,
        bag: &BagRef,
        packed_file: &Path,
        meta: &PushMeta,
    ) -> Result<()> {
        let target_dir = self.object_dir(bag);
        if target_dir.exists() {
            fs::remove_dir_all(&target_dir)?;
        }
        fs::create_dir_all(&target_dir)?;

        fs::copy(packed_file, self.data_path(bag))?;

        self.finalize_existing_bundle(bag, meta)
    }

    async fn bag_info(&self, bag: &BagRef) -> Result<Option<BagInfo>> {
        let meta = self.read_meta(bag)?;
        Ok(Some(BagInfo {
            bundle_hash: meta.bundle_hash,
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            pointcloud: meta.pointcloud,
            mcap_compression: meta.mcap_compression,
            pushed_at: meta.pushed_at,
        }))
    }

    async fn pull(&self, bag: &BagRef, out_packed_file: &Path) -> Result<RemoteDescriptor> {
        let src = self.data_path(bag);
        if !src.exists() {
            return Err(anyhow!("bag not found in folder registry: {}", bag));
        }
        let parent = out_packed_file
            .parent()
            .ok_or_else(|| anyhow!("invalid destination path"))?;
        tokio::fs::create_dir_all(parent).await?;
        tokio::fs::copy(src, out_packed_file).await?;

        let meta = self.read_meta(bag)?;
        Ok(RemoteDescriptor {
            registry_name: self.name.clone(),
            bag: meta.bag,
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
        })
    }

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;
        let mut out = Vec::new();

        for entry in WalkDir::new(&self.root) {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() || path.file_name().and_then(|n| n.to_str()) != Some("metadata.json")
            {
                continue;
            }
            let text = fs::read_to_string(path)
                .with_context(|| format!("failed reading {}", path.display()))?;
            let meta: MetaFile = serde_json::from_str(&text)
                .with_context(|| format!("failed parsing {}", path.display()))?;
            let bag = meta.bag.without_attachment();
            if pattern.matches(&bag.to_string()) {
                out.push(bag);
            }
        }

        Ok(out)
    }

    async fn remove(&self, bag: &BagRef) -> Result<()> {
        let dir = self.object_dir(bag);
        if dir.exists() {
            fs::remove_dir_all(dir)?;
        }
        Ok(())
    }

    async fn write_http_index(&self) -> Result<()> {
        let mut bags = Vec::new();
        for entry in WalkDir::new(&self.root) {
            let entry = entry?;
            let path = entry.path();
            if !path.is_file() || path.file_name().and_then(|n| n.to_str()) != Some("metadata.json")
            {
                continue;
            }
            let text = fs::read_to_string(path)
                .with_context(|| format!("failed reading {}", path.display()))?;
            let meta: MetaFile = serde_json::from_str(&text)
                .with_context(|| format!("failed parsing {}", path.display()))?;
            bags.push(HttpIndexEntry {
                bag: meta.bag.without_attachment(),
                original_bytes: meta.original_bytes,
                packed_bytes: meta.packed_bytes,
                tags: if meta.tags.is_empty() {
                    meta.bag.tags.clone()
                } else {
                    meta.tags
                },
            });
        }
        bags.sort_by_key(|e| e.bag.to_string());
        bags.dedup_by(|a, b| a.bag == b.bag);

        let index = HttpIndexFile { bags };
        let path = self.root.join("index.json");
        fs::write(&path, serde_json::to_vec_pretty(&index)?)
            .with_context(|| format!("failed writing {}", path.display()))?;
        Ok(())
    }

    async fn check_write_access(&self) -> Result<()> {
        let probe = self.root.join(".marina_write_probe");
        fs::create_dir_all(&probe).with_context(|| {
            format!(
                "failed creating write probe directory in folder registry '{}'",
                self.root.display()
            )
        })?;
        fs::remove_dir(&probe).with_context(|| {
            format!(
                "failed removing write probe directory in folder registry '{}'",
                self.root.display()
            )
        })?;
        Ok(())
    }
}
