use std::fs::{self, OpenOptions};
use std::io::{IsTerminal, Write};
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use async_trait::async_trait;
use glob::Pattern;
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use reqwest::Client;
use reqwest::StatusCode;
use serde::Deserialize;

use crate::model::bag_ref::BagRef;
use crate::registry::driver::{PushMeta, RegistryDriver, RemoteDescriptor};

#[derive(Debug, Clone)]
pub struct HttpRegistry {
    pub name: String,
    base_url: String,
    client: Client,
}

#[derive(Debug, Clone, Deserialize)]
struct MetaFile {
    bag: BagRef,
    original_bytes: u64,
    packed_bytes: u64,
}

#[derive(Debug, Clone, Deserialize)]
struct HttpIndexEntry {
    bag: BagRef,
    #[serde(default)]
    original_bytes: Option<u64>,
    #[serde(default)]
    packed_bytes: Option<u64>,
    #[serde(default, rename = "tags")]
    _tags: Vec<String>,
    #[serde(default)]
    bundle_url: Option<String>,
    #[serde(default)]
    metadata_url: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
enum HttpIndexFile {
    Array(Vec<HttpIndexEntry>),
    Object { bags: Vec<HttpIndexEntry> },
}

impl HttpRegistry {
    pub fn from_uri(name: &str, uri: &str) -> Result<Self> {
        if !uri.starts_with("http://") && !uri.starts_with("https://") {
            return Err(anyhow!(
                "http registry URI must start with http:// or https://"
            ));
        }
        let base_url = uri.trim_end_matches('/').to_string();
        Ok(Self {
            name: name.to_string(),
            base_url,
            client: Client::builder()
                .connect_timeout(Duration::from_secs(30))
                .timeout(Duration::from_secs(60 * 60))
                .build()?,
        })
    }

    fn index_url(&self) -> String {
        format!("{}/index.json", self.base_url)
    }

    fn object_base_url(&self, bag: &BagRef) -> String {
        format!("{}/{}", self.base_url, bag.object_path())
    }

    fn default_bundle_url(&self, bag: &BagRef) -> String {
        format!("{}/bundle.marina.tar.gz", self.object_base_url(bag))
    }

    fn default_metadata_url(&self, bag: &BagRef) -> String {
        format!("{}/metadata.json", self.object_base_url(bag))
    }

    async fn fetch_index(&self) -> Result<Vec<HttpIndexEntry>> {
        let resp = self
            .client
            .get(self.index_url())
            .send()
            .await
            .context("failed to fetch http registry index.json")?;

        if resp.status() == StatusCode::NOT_FOUND {
            return Ok(Vec::new());
        }

        let text = resp
            .error_for_status()
            .context("http registry index request failed")?
            .text()
            .await
            .context("failed reading http registry index response")?;

        let parsed: HttpIndexFile =
            serde_json::from_str(&text).context("failed parsing http registry index.json")?;
        Ok(match parsed {
            HttpIndexFile::Array(items) => items,
            HttpIndexFile::Object { bags } => bags,
        })
    }

    async fn find_index_entry(&self, bag: &BagRef) -> Result<Option<HttpIndexEntry>> {
        let target = bag.without_attachment();
        for item in self.fetch_index().await? {
            if item.bag.without_attachment() == target {
                return Ok(Some(item));
            }
        }
        Ok(None)
    }

    async fn download_file_with_progress(
        &self,
        url: &str,
        out: &Path,
        title: &str,
        size_hint: Option<u64>,
    ) -> Result<u64> {
        let mut existing = fs::metadata(out).map(|m| m.len()).unwrap_or(0);
        let mut req = self.client.get(url);
        if existing > 0 {
            req = req.header(reqwest::header::RANGE, format!("bytes={existing}-"));
        }
        let resp = req
            .send()
            .await
            .with_context(|| format!("failed downloading {}", title))?;
        let status = resp.status();
        if status == StatusCode::RANGE_NOT_SATISFIABLE {
            // Local file is likely complete already.
            return Ok(existing);
        }
        let resp = resp
            .error_for_status()
            .with_context(|| format!("download failed for {}", title))?;

        // If server ignored range and returned full content, restart from scratch.
        if existing > 0 && status != StatusCode::PARTIAL_CONTENT {
            existing = 0;
        }

        let total = resp
            .content_length()
            .map(|n| n + existing)
            .or(size_hint)
            .unwrap_or(0);
        let hidden = !std::io::stdout().is_terminal();
        let pb = if total > 0 {
            let pb = ProgressBar::new(total);
            if hidden {
                pb.set_draw_target(ProgressDrawTarget::hidden());
            }
            pb.set_style(
                ProgressStyle::with_template(
                    "{msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes} {bytes_per_sec} ({eta})",
                )
                .unwrap_or_else(|_| ProgressStyle::default_bar()),
            );
            pb.set_message(title.to_string());
            if existing > 0 {
                pb.set_position(existing.min(total));
            }
            Some(pb)
        } else {
            let pb = ProgressBar::new_spinner();
            if hidden {
                pb.set_draw_target(ProgressDrawTarget::hidden());
            }
            pb.set_style(
                ProgressStyle::with_template("{spinner} {msg} {bytes} {bytes_per_sec}")
                    .unwrap_or_else(|_| ProgressStyle::default_spinner())
                    .tick_chars("|/-\\ "),
            );
            pb.set_message(title.to_string());
            pb.enable_steady_tick(std::time::Duration::from_millis(100));
            Some(pb)
        };

        if let Some(parent) = out.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut file = if existing > 0 {
            OpenOptions::new()
                .append(true)
                .open(out)
                .with_context(|| format!("failed opening output file {}", out.display()))?
        } else {
            fs::File::create(out)
                .with_context(|| format!("failed creating output file {}", out.display()))?
        };

        let mut downloaded = existing;
        let mut resp = resp;
        while let Some(chunk) = resp.chunk().await? {
            file.write_all(&chunk)?;
            downloaded += chunk.len() as u64;
            if let Some(pb) = &pb {
                pb.inc(chunk.len() as u64);
            }
        }
        if let Some(pb) = pb {
            pb.finish_and_clear();
        }
        Ok(downloaded)
    }

    async fn fetch_metadata(&self, url: &str) -> Result<MetaFile> {
        let text = self
            .client
            .get(url)
            .send()
            .await
            .with_context(|| format!("failed downloading metadata {}", url))?
            .error_for_status()
            .with_context(|| format!("metadata request failed for {}", url))?
            .text()
            .await
            .context("failed reading metadata response")?;
        let meta: MetaFile = serde_json::from_str(&text)
            .with_context(|| format!("failed parsing metadata json from {}", url))?;
        Ok(meta)
    }
}

#[async_trait]
impl RegistryDriver for HttpRegistry {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    async fn push(
        &self,
        _registry_name: &str,
        _bag: &BagRef,
        _packed_file: &Path,
        _meta: &PushMeta,
    ) -> Result<()> {
        Err(anyhow!(
            "http registry '{}' is read-only: push is not supported",
            self.name
        ))
    }

    async fn pull(&self, bag: &BagRef, out_packed_file: &Path) -> Result<RemoteDescriptor> {
        let target = bag.without_attachment();
        let entry = self.find_index_entry(&target).await?;

        let bundle_url = entry
            .as_ref()
            .and_then(|e| e.bundle_url.clone())
            .unwrap_or_else(|| self.default_bundle_url(&target));
        let metadata_url = entry
            .as_ref()
            .and_then(|e| e.metadata_url.clone())
            .unwrap_or_else(|| self.default_metadata_url(&target));

        let size_hint = entry.as_ref().and_then(|e| e.packed_bytes);
        let downloaded = self
            .download_file_with_progress(
                &bundle_url,
                out_packed_file,
                &format!("{}", target),
                size_hint,
            )
            .await?;

        let descriptor = match self.fetch_metadata(&metadata_url).await {
            Ok(meta) => RemoteDescriptor {
                registry_name: self.name.clone(),
                bag: meta.bag.without_attachment(),
                original_bytes: meta.original_bytes,
                packed_bytes: meta.packed_bytes,
            },
            Err(_) => RemoteDescriptor {
                registry_name: self.name.clone(),
                bag: target,
                original_bytes: entry
                    .as_ref()
                    .and_then(|e| e.original_bytes)
                    .unwrap_or(downloaded),
                packed_bytes: entry
                    .as_ref()
                    .and_then(|e| e.packed_bytes)
                    .unwrap_or(downloaded),
            },
        };

        Ok(descriptor)
    }

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;
        let items = self.fetch_index().await?;
        if items.is_empty() {
            return Err(anyhow!(
                "HTTP registry '{}' has no index.json. Use an exact dataset reference to pull",
                self.name
            ));
        }

        let mut out = Vec::new();
        for item in items {
            let bag = item.bag.without_attachment();
            if pattern.matches(&bag.to_string()) {
                out.push(bag);
            }
        }
        out.sort_by_key(|b| b.to_string());
        out.dedup();
        Ok(out)
    }

    async fn remove(&self, _bag: &BagRef) -> Result<()> {
        Err(anyhow!(
            "http registry '{}' is read-only: remove is not supported",
            self.name
        ))
    }

    async fn check_connection(&self) -> Result<()> {
        self.client
            .get(&self.base_url)
            .timeout(Duration::from_secs(5))
            .send()
            .await
            .context("failed checking http registry connectivity")?
            .error_for_status()
            .context("http registry connectivity check returned error")?;
        Ok(())
    }

    async fn check_write_access(&self) -> Result<()> {
        Err(anyhow!(
            "http registry '{}' is read-only: push is not supported",
            self.name
        ))
    }
}
