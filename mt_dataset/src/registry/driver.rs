use std::any::Any;
use std::path::Path;

use anyhow::{Result, anyhow};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::model::bag_ref::BagRef;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RemoteDescriptor {
    pub registry_name: String,
    pub bag: BagRef,
    pub original_bytes: u64,
    pub packed_bytes: u64,
}

/// Metadata written alongside the bundle when pushing.
#[derive(Debug, Clone)]
pub struct PushMeta {
    pub original_bytes: u64,
    pub packed_bytes: u64,
    /// Short SHA-256 of the packed bundle (first 12 hex chars).
    pub bundle_hash: String,
    /// Pointcloud encoding: "lossless", "lossy", or "disabled".
    pub pointcloud: String,
    /// MCAP chunk compression: "none", "zstd", or "lz4".
    pub mcap_compression: String,
    /// Unix timestamp (seconds) when this bundle was pushed.
    pub pushed_at: u64,
}

/// Per-bag information readable from a remote registry.
#[derive(Debug, Clone)]
pub struct BagInfo {
    pub bundle_hash: Option<String>,
    pub original_bytes: u64,
    pub packed_bytes: u64,
    pub pointcloud: Option<String>,
    pub mcap_compression: Option<String>,
    pub pushed_at: Option<u64>,
}

#[async_trait]
pub trait RegistryDriver: Send + Sync {
    fn as_any(&self) -> &dyn Any;

    /// Return the registry's byte-range streaming capability, when available.
    #[cfg(feature = "minot-registry")]
    fn as_streaming(&self) -> Option<&dyn StreamingDriver> {
        None
    }

    async fn push(
        &self,
        registry_name: &str,
        bag: &BagRef,
        packed_file: &Path,
        meta: &PushMeta,
    ) -> Result<()>;

    async fn pull(&self, bag: &BagRef, out_packed_file: &Path) -> Result<RemoteDescriptor>;

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>>;

    async fn remove(&self, bag: &BagRef) -> Result<()>;

    /// Fetch lightweight metadata for a specific bag. Returns `None` if unsupported.
    async fn bag_info(&self, _bag: &BagRef) -> Result<Option<BagInfo>> {
        Ok(None)
    }

    /// List all matching bags together with their metadata in one operation.
    /// Drivers that can batch this more efficiently should override the default.
    async fn list_with_info(&self, filter: &str) -> Result<Vec<(BagRef, Option<BagInfo>)>> {
        let bags = self.list(filter).await?;
        let mut result = Vec::new();
        for bag in bags {
            let info = self.bag_info(&bag).await.ok().flatten();
            result.push((bag, info));
        }
        Ok(result)
    }

    async fn write_http_index(&self) -> Result<()> {
        Err(anyhow!(
            "http index generation is not supported for this registry type"
        ))
    }

    async fn check_connection(&self) -> Result<()> {
        Ok(())
    }

    async fn check_write_access(&self) -> Result<()> {
        self.check_connection().await
    }
}

/// Optional capability implemented by registries that can open datasets
/// without first materialising them locally.
#[cfg(feature = "minot-registry")]
#[async_trait]
pub trait StreamingDriver: RegistryDriver {
    async fn open_dataset(&self, bag: &BagRef) -> Result<crate::registry::minot::RemoteDataset>;
}
