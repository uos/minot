//! `marina serve` — expose a local registry over a Minot network.
//!
//! The server is a thin adapter, deliberately. It holds an ordinary
//! [`RegistryDriver`] — a folder, an SSH registry, anything already configured —
//! and answers requests by delegating to it. That is what makes a `minot://`
//! registry follow the behavior of the registry behind it.
//!
//! # Binding
//!
//! Minot is given no authentication of its own, so this binds **loopback only**
//! and expects to be reached through an SSH tunnel. See the streaming plan's
//! auth section: the client forwards a local port to the server's loopback port
//! using the credentials the SSH registry already manages, so users configure
//! them once.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::{Duration, SystemTime};

use anyhow::{Context, Result};
use mt_flow::{BytesSource, FlowConfig, FlowSender};
use mt_pubsub::{CoordMode, Node, NodeConfig, Qos};
use mt_service::ServiceServer;

use crate::registry::driver::RegistryDriver;
use crate::registry::minot::protocol::{
    PROTOCOL_VERSION, Request, Response, WireBagInfo, WireBagRef, WireFile, WireManifestFile,
    WirePushMeta, service_topic, version_mismatch,
};

/// How a served registry is reached and what it is allowed to do.
pub struct ServeOptions {
    /// Name clients use to address this registry. Also namespaces the topics.
    pub registry: String,
    /// Restrict Minot to this machine. True unless you have arranged transport
    /// security yourself — see the module docs.
    pub local_only: bool,
    /// Chunking and window sizing for bundle transfers.
    pub flow: FlowConfig,
    /// Where datasets are unpacked so their bytes can be served by range.
    ///
    /// Defaults to a `serve/` directory under the marina cache. Materialising
    /// here. This keeps the server's working set
    /// separate from whatever the same machine pulled for its own use.
    pub materialize_root: Option<std::path::PathBuf>,
    /// Largest range a client may ask for in one request, as a guard against a
    /// misbehaving or hostile client asking for a gigabyte.
    pub max_range_bytes: u32,
    /// Accept staged dataset uploads. Off by default because Minot itself does
    /// has no client authentication. SSH provides the authorization boundary.
    pub allow_write: bool,
    /// Maximum idle age of unpacked streaming datasets and abandoned staging
    /// archives before the periodic sweep removes them.
    pub cache_max_age: Duration,
}

impl ServeOptions {
    pub fn new(registry: impl Into<String>) -> Self {
        Self {
            registry: registry.into(),
            local_only: true,
            flow: FlowConfig::default(),
            materialize_root: None,
            max_range_bytes: 8 * 1024 * 1024,
            allow_write: false,
            cache_max_age: Duration::from_secs(96 * 60 * 60),
        }
    }
}

/// Serve `driver` until the process is stopped.
pub async fn serve(driver: Arc<dyn RegistryDriver>, options: ServeOptions) -> Result<()> {
    if options.local_only {
        mt_sea::network::set_local_only(true);
    } else {
        // Direct clients address this server explicitly. Multicast discovery
        // would merge it with unrelated Minot runs on the same LAN.
        mt_sea::network::set_unicast_only(true);
        log::warn!(
            "marina serve is not restricted to this machine. Minot provides no \
             authentication, so anything that can reach this port can {} the \
             '{}' registry. Prefer an SSH tunnel to a loopback-bound server.",
            if options.allow_write {
                "read and write"
            } else {
                "read"
            },
            options.registry
        );
    }

    driver
        .check_connection()
        .await
        .context("the registry being served is not reachable")?;
    if options.allow_write {
        driver
            .check_write_access()
            .await
            .context("the registry being served is not writable")?;
    }

    // TryReliable, never Reliable: a client that dies must not torpedo the
    // server, and the server must survive its own coordinator restarting.
    let node = Arc::new(
        Node::create(
            NodeConfig::new(format!("marina_serve_{}", options.registry))
                .mode(Qos::TryReliable)
                .coord_mode(CoordMode::Start)
                .wan(),
        )
        .await
        .context("could not join the Minot network")?,
    );

    let topic = service_topic(&options.registry);
    let server = ServiceServer::<Request, Response>::new(Arc::clone(&node), topic.clone())
        .await
        .with_context(|| format!("could not listen on '{topic}'"))?;

    log::info!(
        "marina serve: exposing registry '{}' on '{}' (protocol {}.{})",
        options.registry,
        topic,
        PROTOCOL_VERSION.0,
        PROTOCOL_VERSION.1
    );

    let materialize_root = match options.materialize_root {
        Some(root) => root,
        None => crate::storage::config::cache_dir()
            .context("could not locate the cache directory")?
            .join("serve"),
    };
    std::fs::create_dir_all(&materialize_root).with_context(|| {
        format!(
            "could not create the serving directory {}",
            materialize_root.display()
        )
    })?;

    let handler = Arc::new(RequestHandler {
        driver,
        node: Arc::clone(&node),
        registry: options.registry.clone(),
        flow: options.flow,
        materialize_root,
        max_range_bytes: options.max_range_bytes,
        allow_write: options.allow_write,
        cache_max_age: options.cache_max_age,
        materialize_lock: tokio::sync::Mutex::new(()),
        materializing: tokio::sync::Mutex::new(HashSet::new()),
        materialize_errors: tokio::sync::Mutex::new(HashMap::new()),
        last_access: tokio::sync::Mutex::new(HashMap::new()),
        cache_sweep_lock: tokio::sync::RwLock::new(()),
        write_lock: tokio::sync::Mutex::new(()),
    });

    let sweeper = Arc::clone(&handler);
    tokio::spawn(async move {
        sweeper.sweep_cache().await;
        loop {
            tokio::time::sleep(Duration::from_secs(60 * 60)).await;
            sweeper.sweep_cache().await;
        }
    });

    ServiceServer::start(
        server,
        Arc::new(move |request| {
            let handler = Arc::clone(&handler);
            async move { handler.handle(request).await }
        }),
    )
    .await;

    Ok(())
}

struct RequestHandler {
    driver: Arc<dyn RegistryDriver>,
    node: Arc<Node>,
    registry: String,
    flow: FlowConfig,
    materialize_root: std::path::PathBuf,
    max_range_bytes: u32,
    allow_write: bool,
    cache_max_age: Duration,
    /// Serialises the disk-heavy restore itself.
    materialize_lock: tokio::sync::Mutex<()>,
    /// Dataset keys currently restoring, so every poll returns immediately and
    /// only the first one starts work.
    materializing: tokio::sync::Mutex<HashSet<String>>,
    /// The next poll receives any background materialization failure.
    materialize_errors: tokio::sync::Mutex<HashMap<String, String>>,
    /// Last request time for datasets used since this server started.
    last_access: tokio::sync::Mutex<HashMap<String, SystemTime>>,
    /// Keeps a sweep from removing a dataset while a request is using it.
    cache_sweep_lock: tokio::sync::RwLock<()>,
    /// Serialises staging mutations and commits. Upload traffic is already
    /// sequential per client. It also prevents two clients racing one tag.
    write_lock: tokio::sync::Mutex<()>,
}

impl RequestHandler {
    async fn record_access(&self, bag: &crate::model::bag_ref::BagRef) {
        let key = bag.without_attachment().cache_key();
        let now = SystemTime::now();
        let persist = self
            .last_access
            .lock()
            .await
            .insert(key, now)
            .is_none_or(|previous| {
                now.duration_since(previous).unwrap_or_default() >= Duration::from_secs(60 * 60)
            });
        if persist {
            let marker = self.dataset_dir(bag).join(".last-access");
            if marker.parent().is_some_and(|parent| parent.is_dir()) {
                let _ = tokio::fs::write(marker, []).await;
            }
        }
    }

    async fn sweep_cache(&self) {
        let last_access = self.last_access.lock().await.clone();
        let materialize_root = self.materialize_root.clone();
        let temp_root = std::env::temp_dir();
        let cache_max_age = self.cache_max_age;
        let plan = {
            let _guard = self.cache_sweep_lock.write().await;
            prepare_cache_sweep(&materialize_root, &temp_root, cache_max_age, &last_access)
        };
        let result = match plan {
            Ok(plan) => tokio::task::spawn_blocking(move || remove_sweep_paths(plan)).await,
            Err(error) => {
                log::warn!("marina serve: cache sweep failed: {error}");
                return;
            }
        };
        match result {
            Ok(Ok(summary)) if summary.entries > 0 => log::info!(
                "marina serve: removed {} expired streaming cache entries ({})",
                summary.entries,
                format_bytes(summary.bytes)
            ),
            Ok(Ok(_)) => {}
            Ok(Err(error)) => log::warn!("marina serve: cache sweep failed: {error}"),
            Err(error) => log::warn!("marina serve: cache sweep stopped: {error}"),
        }
    }

    async fn handle(self: &Arc<Self>, request: Request) -> Result<Response, String> {
        match request {
            Request::Hello { major, minor } => {
                if let Some(reason) = version_mismatch(major, minor) {
                    log::warn!("marina serve: refusing a client — {reason}");
                    return Err(reason);
                }
                Ok(Response::Hello {
                    major: PROTOCOL_VERSION.0,
                    minor: PROTOCOL_VERSION.1,
                    registry: self.registry.clone(),
                })
            }

            Request::List { pattern } => {
                let bags = self
                    .driver
                    .list(&pattern)
                    .await
                    .map_err(|error| format!("list failed: {error}"))?;
                Ok(Response::List(
                    bags.iter().map(WireBagRef::from).collect::<Vec<_>>(),
                ))
            }

            Request::BagInfo { bag } => {
                let bag = bag.into();
                let info = self
                    .driver
                    .bag_info(&bag)
                    .await
                    .map_err(|error| format!("bag_info failed: {error}"))?;
                Ok(Response::BagInfo(info.as_ref().map(WireBagInfo::from)))
            }

            Request::PullBegin { bag } => self.begin_pull(bag.into()).await,

            Request::Stat { bag } => self.stat(bag.into()).await,

            Request::ReadRange {
                bag,
                path,
                offset,
                len,
            } => self.read_range(bag.into(), &path, offset, len).await,

            Request::CheckWrite => {
                self.ensure_writes_enabled()?;
                Ok(Response::WriteAllowed)
            }

            Request::Remove { bag } => self.remove(bag.into()).await,

            Request::BeginPush {
                bag,
                packed_bytes,
                bundle_hash,
            } => {
                self.begin_push(bag.into(), packed_bytes, &bundle_hash)
                    .await
            }

            Request::PushRange { bag, offset, data } => {
                self.push_range(bag.into(), offset, data).await
            }

            Request::CommitPush { bag, meta } => self.commit_push(bag.into(), meta).await,

            Request::BeginWrite { bag } => self.begin_write(bag.into()).await,

            Request::WriteRange {
                bag,
                path,
                offset,
                data,
            } => self.write_range(bag.into(), &path, offset, data).await,

            Request::CommitWrite { bag, files } => self.commit_write(bag.into(), files).await,
        }
    }

    /// Where a dataset is unpacked on this server.
    fn dataset_dir(&self, bag: &crate::model::bag_ref::BagRef) -> std::path::PathBuf {
        self.materialize_root
            .join(bag.without_attachment().cache_key())
    }

    /// Ensure the dataset is unpacked here, pulling it from the backing
    /// registry the first time.
    async fn materialize(
        &self,
        bag: &crate::model::bag_ref::BagRef,
    ) -> Result<std::path::PathBuf, String> {
        let _cache_guard = self.cache_sweep_lock.read().await;
        let ready = self.dataset_dir(bag).join("ready");
        if ready.is_dir() {
            return Ok(ready);
        }

        let _guard = self.materialize_lock.lock().await;
        // Checked again under the lock: another request may have done it while
        // this one waited.
        if ready.is_dir() {
            return Ok(ready);
        }

        log::info!("marina serve: materialising '{bag}' for range reads");
        let staging = tempfile::Builder::new()
            .prefix("marina-materialize-")
            .suffix(".tar.gz")
            .tempfile()
            .map_err(|error| format!("could not stage '{bag}': {error}"))?;
        let descriptor = self
            .driver
            .pull(bag, staging.path())
            .await
            .map_err(|error| format!("could not read '{bag}' from the served registry: {error}"))?;
        log::info!(
            "marina serve: restored {} packed bytes for '{bag}', unpacking",
            descriptor.packed_bytes
        );

        // Unpacked beside the destination and renamed, so an interrupted
        // materialisation never leaves a half-written tree that looks ready.
        let parent = self.dataset_dir(bag);
        let incoming = parent.join(".incoming");
        if incoming.exists() {
            let _ = std::fs::remove_dir_all(&incoming);
        }
        std::fs::create_dir_all(&incoming)
            .map_err(|error| format!("could not prepare '{bag}': {error}"))?;

        let staging_path = staging.path().to_path_buf();
        let incoming_for_task = incoming.clone();
        // Unpacking is synchronous CPU and disk work. A blocking task keeps the
        // async worker available for other clients.
        tokio::task::spawn_blocking(move || {
            crate::io::pack::unpack_bag(&staging_path, &incoming_for_task)
        })
        .await
        .map_err(|error| format!("unpacking '{bag}' panicked: {error}"))?
        .map_err(|error| format!("could not unpack '{bag}': {error}"))?;

        std::fs::rename(&incoming, &ready)
            .map_err(|error| format!("could not install '{bag}': {error}"))?;
        let _ = std::fs::write(parent.join(".last-access"), []);
        log::info!("marina serve: '{bag}' is ready for range reads");
        Ok(ready)
    }

    async fn stat(
        self: &Arc<Self>,
        bag: crate::model::bag_ref::BagRef,
    ) -> Result<Response, String> {
        self.record_access(&bag).await;
        let _cache_guard = self.cache_sweep_lock.read().await;
        let ready = self.dataset_dir(&bag).join("ready");
        if !ready.is_dir() {
            let key = bag.without_attachment().to_string();
            if let Some(error) = self.materialize_errors.lock().await.get(&key).cloned() {
                return Err(format!("materialising '{bag}' failed: {error}"));
            }

            let should_start = self.materializing.lock().await.insert(key.clone());
            if should_start {
                let handler = Arc::clone(self);
                let bag_for_task = bag.clone();
                tokio::spawn(async move {
                    if let Err(error) = handler.materialize(&bag_for_task).await {
                        log::error!("marina serve: materialising '{bag_for_task}' failed: {error}");
                        handler
                            .materialize_errors
                            .lock()
                            .await
                            .insert(key.clone(), error);
                    }
                    handler.materializing.lock().await.remove(&key);
                });
            }

            return Ok(Response::Materializing {
                message: if should_start {
                    format!("restoring '{bag}' on the server")
                } else {
                    format!("still restoring '{bag}' on the server")
                },
            });
        }

        let mut files = Vec::new();
        for entry in walkdir::WalkDir::new(&ready).follow_links(false) {
            let entry = entry.map_err(|error| format!("could not list '{bag}': {error}"))?;
            if !entry.file_type().is_file() {
                continue;
            }
            let relative = entry
                .path()
                .strip_prefix(&ready)
                .map_err(|error| format!("could not list '{bag}': {error}"))?
                .to_string_lossy()
                .replace('\\', "/");
            let size = entry
                .metadata()
                .map_err(|error| format!("could not stat '{relative}': {error}"))?
                .len();
            files.push(WireFile {
                path: relative,
                size,
            });
        }
        files.sort_by(|a, b| a.path.cmp(&b.path));

        // A sqlite3 bag needs a real file for rusqlite to open, so it cannot be
        // read by range. Say so plainly and let the client fall back to a pull
        // before the client starts reading.
        let has_db3 = files.iter().any(|file| file.path.ends_with(".db3"));
        let (streamable, reason) = if has_db3 {
            (
                false,
                Some(
                    "sqlite3 bags require a local file. Pull this dataset before opening it"
                        .to_string(),
                ),
            )
        } else {
            (true, None)
        };

        Ok(Response::Stat {
            files,
            streamable,
            reason,
        })
    }

    async fn read_range(
        &self,
        bag: crate::model::bag_ref::BagRef,
        path: &str,
        offset: u64,
        len: u32,
    ) -> Result<Response, String> {
        self.record_access(&bag).await;
        if len > self.max_range_bytes {
            return Err(format!(
                "requested range of {len} bytes exceeds this server's limit of {}",
                self.max_range_bytes
            ));
        }
        let ready = self.materialize(&bag).await?;
        let _cache_guard = self.cache_sweep_lock.read().await;
        let target = safe_join(&ready, path)?;

        let data = tokio::task::spawn_blocking(move || -> std::io::Result<Vec<u8>> {
            use std::io::{Read, Seek, SeekFrom};
            let mut file = std::fs::File::open(&target)?;
            let size = file.metadata()?.len();
            if offset >= size {
                return Ok(Vec::new());
            }
            let readable = (size - offset).min(len as u64) as usize;
            let mut buffer = vec![0u8; readable];
            file.seek(SeekFrom::Start(offset))?;
            file.read_exact(&mut buffer)?;
            Ok(buffer)
        })
        .await
        .map_err(|error| format!("reading '{path}' panicked: {error}"))?
        .map_err(|error| format!("could not read '{path}': {error}"))?;

        Ok(Response::ReadRange { data })
    }

    fn ensure_writes_enabled(&self) -> Result<(), String> {
        if self.allow_write {
            Ok(())
        } else {
            Err("writes require `marina serve --allow-write`".to_string())
        }
    }

    async fn remove(&self, bag: crate::model::bag_ref::BagRef) -> Result<Response, String> {
        self.ensure_writes_enabled()?;
        let _guard = self.write_lock.lock().await;
        self.driver
            .remove(&bag)
            .await
            .map_err(|error| format!("could not remove '{bag}' from backing registry: {error}"))?;

        // The unpacked serving copy is a cache, not an independent dataset.
        // Keeping it after the backing object is deleted would make the same
        // server continue to stream data that list/pull says no longer exists.
        let materialized = self.dataset_dir(&bag);
        if materialized.exists() {
            std::fs::remove_dir_all(&materialized).map_err(|error| {
                format!(
                    "removed '{bag}' from the backing registry but could not clear stale serving cache {}: {error}",
                    materialized.display()
                )
            })?;
        }
        self.materialize_errors
            .lock()
            .await
            .remove(&bag.without_attachment().to_string());
        Ok(Response::Removed)
    }

    fn packed_push_path(&self, bag: &crate::model::bag_ref::BagRef) -> std::path::PathBuf {
        self.dataset_dir(bag).join(".push-incoming.bundle")
    }

    fn packed_push_identity_path(&self, bag: &crate::model::bag_ref::BagRef) -> std::path::PathBuf {
        self.dataset_dir(bag).join(".push-incoming.identity")
    }

    async fn begin_push(
        &self,
        bag: crate::model::bag_ref::BagRef,
        packed_bytes: u64,
        bundle_hash: &str,
    ) -> Result<Response, String> {
        self.ensure_writes_enabled()?;
        let _guard = self.write_lock.lock().await;
        let path = self.packed_push_path(&bag);
        let identity_path = self.packed_push_identity_path(&bag);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|error| format!("could not prepare packed upload '{bag}': {error}"))?;
        }
        let identity = format!("{packed_bytes}\n{bundle_hash}\n");
        let same_upload = std::fs::read_to_string(&identity_path)
            .map(|existing| existing == identity)
            .unwrap_or(false);
        if !same_upload {
            let _ = std::fs::remove_file(&path);
            std::fs::write(&identity_path, identity)
                .map_err(|error| format!("could not identify packed upload '{bag}': {error}"))?;
        }
        let next_offset = std::fs::metadata(&path)
            .map(|metadata| metadata.len())
            .unwrap_or(0);
        if next_offset > packed_bytes {
            std::fs::remove_file(&path)
                .map_err(|error| format!("could not reset packed upload '{bag}': {error}"))?;
            return Ok(Response::PushStatus { next_offset: 0 });
        }
        Ok(Response::PushStatus { next_offset })
    }

    async fn push_range(
        &self,
        bag: crate::model::bag_ref::BagRef,
        offset: u64,
        data: Vec<u8>,
    ) -> Result<Response, String> {
        use std::io::{Read, Seek, SeekFrom, Write};

        self.ensure_writes_enabled()?;
        if data.len() > self.max_range_bytes as usize {
            return Err(format!(
                "packed upload range of {} bytes exceeds this server's limit of {}",
                data.len(),
                self.max_range_bytes
            ));
        }
        let _guard = self.write_lock.lock().await;
        let path = self.packed_push_path(&bag);
        if !self.packed_push_identity_path(&bag).is_file() {
            return Err(format!("no packed upload was begun for '{bag}'"));
        }
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&path)
            .map_err(|error| format!("could not open packed upload '{bag}': {error}"))?;
        let current = file
            .metadata()
            .map_err(|error| format!("could not inspect packed upload '{bag}': {error}"))?
            .len();
        if offset < current {
            let overlap = (current - offset).min(data.len() as u64) as usize;
            let mut existing = vec![0u8; overlap];
            file.seek(SeekFrom::Start(offset))
                .and_then(|_| file.read_exact(&mut existing))
                .map_err(|error| format!("could not verify repeated packed range: {error}"))?;
            if existing != data[..overlap] {
                return Err(format!(
                    "packed upload for '{bag}' differs at byte {offset}"
                ));
            }
            if overlap < data.len() {
                file.seek(SeekFrom::End(0))
                    .and_then(|_| file.write_all(&data[overlap..]))
                    .and_then(|_| file.flush())
                    .map_err(|error| format!("could not append packed upload: {error}"))?;
            }
            return Ok(Response::PushAck {
                next_offset: current + (data.len() - overlap) as u64,
            });
        }
        if offset > current {
            return Ok(Response::PushAck {
                next_offset: current,
            });
        }
        file.seek(SeekFrom::End(0))
            .and_then(|_| file.write_all(&data))
            .and_then(|_| file.flush())
            .map_err(|error| format!("could not append packed upload: {error}"))?;
        Ok(Response::PushAck {
            next_offset: current + data.len() as u64,
        })
    }

    async fn commit_push(
        &self,
        bag: crate::model::bag_ref::BagRef,
        wire_meta: WirePushMeta,
    ) -> Result<Response, String> {
        use sha2::{Digest, Sha256};
        use std::io::Read;

        self.ensure_writes_enabled()?;
        let _guard = self.write_lock.lock().await;
        let path = self.packed_push_path(&bag);
        let meta: crate::registry::driver::PushMeta = wire_meta.into();
        let actual_size = std::fs::metadata(&path)
            .map_err(|error| format!("could not inspect completed packed upload '{bag}': {error}"))?
            .len();
        if actual_size != meta.packed_bytes {
            return Err(format!(
                "cannot commit packed '{bag}': received {actual_size} of {} bytes",
                meta.packed_bytes
            ));
        }
        let mut file = std::fs::File::open(&path)
            .map_err(|error| format!("could not open completed packed upload '{bag}': {error}"))?;
        let mut hasher = Sha256::new();
        let mut buffer = vec![0u8; 1024 * 1024];
        loop {
            let read = file
                .read(&mut buffer)
                .map_err(|error| format!("could not hash packed upload '{bag}': {error}"))?;
            if read == 0 {
                break;
            }
            hasher.update(&buffer[..read]);
        }
        let actual_hash = hasher
            .finalize()
            .iter()
            .take(6)
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        if actual_hash != meta.bundle_hash {
            return Err(format!(
                "cannot commit packed '{bag}': bundle hash is {actual_hash}, expected {}",
                meta.bundle_hash
            ));
        }
        self.driver
            .push(&self.registry, &bag, &path, &meta)
            .await
            .map_err(|error| format!("could not publish packed '{bag}': {error}"))?;

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(self.packed_push_identity_path(&bag));
        let ready = self.dataset_dir(&bag).join("ready");
        if ready.exists() {
            std::fs::remove_dir_all(&ready)
                .map_err(|error| format!("could not invalidate old stream cache: {error}"))?;
        }
        self.materialize_errors
            .lock()
            .await
            .remove(&bag.without_attachment().to_string());
        Ok(Response::PushCommitted)
    }

    fn write_staging_dir(&self, bag: &crate::model::bag_ref::BagRef) -> std::path::PathBuf {
        self.dataset_dir(bag).join(".write-incoming")
    }

    async fn begin_write(&self, bag: crate::model::bag_ref::BagRef) -> Result<Response, String> {
        self.ensure_writes_enabled()?;
        let _guard = self.write_lock.lock().await;
        let staging = self.write_staging_dir(&bag);
        std::fs::create_dir_all(&staging)
            .map_err(|error| format!("could not stage '{bag}': {error}"))?;
        let files = list_file_sizes(&staging)
            .map_err(|error| format!("could not inspect staged upload '{bag}': {error}"))?;
        Ok(Response::WriteStatus { files })
    }

    async fn write_range(
        &self,
        bag: crate::model::bag_ref::BagRef,
        path: &str,
        offset: u64,
        data: Vec<u8>,
    ) -> Result<Response, String> {
        use std::io::{Read, Seek, SeekFrom, Write};

        self.ensure_writes_enabled()?;
        if data.len() > self.max_range_bytes as usize {
            return Err(format!(
                "upload range of {} bytes exceeds this server's limit of {}",
                data.len(),
                self.max_range_bytes
            ));
        }
        let _guard = self.write_lock.lock().await;
        let staging = self.write_staging_dir(&bag);
        std::fs::create_dir_all(&staging)
            .map_err(|error| format!("could not stage '{bag}': {error}"))?;
        let target = safe_join(&staging, path)?;
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|error| format!("could not create upload directory: {error}"))?;
        }
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&target)
            .map_err(|error| format!("could not open staged file '{path}': {error}"))?;
        let current = file
            .metadata()
            .map_err(|error| format!("could not inspect staged file '{path}': {error}"))?
            .len();

        // A client may retry after losing the acknowledgement. Verify the
        // overlap, then append only bytes the server does not already have.
        if offset < current {
            let overlap = (current - offset).min(data.len() as u64) as usize;
            let mut existing = vec![0u8; overlap];
            file.seek(SeekFrom::Start(offset))
                .and_then(|_| file.read_exact(&mut existing))
                .map_err(|error| {
                    format!("could not verify repeated range for '{path}': {error}")
                })?;
            if existing != data[..overlap] {
                return Err(format!(
                    "staged file '{path}' differs at offset {offset}. Use a new Marina tag or clear the interrupted upload"
                ));
            }
            if overlap == data.len() {
                return Ok(Response::WriteAck {
                    next_offset: current,
                });
            }
            file.seek(SeekFrom::End(0))
                .and_then(|_| file.write_all(&data[overlap..]))
                .and_then(|_| file.flush())
                .map_err(|error| format!("could not append staged file '{path}': {error}"))?;
            return Ok(Response::WriteAck {
                next_offset: current + (data.len() - overlap) as u64,
            });
        }

        if offset > current {
            // The watermark is authoritative. Returning it lets a resumed
            // client repair the missing prefix without creating a sparse hole.
            return Ok(Response::WriteAck {
                next_offset: current,
            });
        }
        file.seek(SeekFrom::End(0))
            .and_then(|_| file.write_all(&data))
            .and_then(|_| file.flush())
            .map_err(|error| format!("could not append staged file '{path}': {error}"))?;
        Ok(Response::WriteAck {
            next_offset: current + data.len() as u64,
        })
    }

    async fn commit_write(
        &self,
        bag: crate::model::bag_ref::BagRef,
        files: Vec<WireManifestFile>,
    ) -> Result<Response, String> {
        use crate::io::mcap_transform::{McapChunkCompression, PointCloudCompressionMode};
        use crate::io::pack::{ArchiveCompression, PackOptions};
        use crate::registry::driver::PushMeta;
        use crate::storage::cache::MirrorFile;
        use sha2::{Digest, Sha256};

        self.ensure_writes_enabled()?;
        let _guard = self.write_lock.lock().await;
        let parent = self.dataset_dir(&bag);
        let staging = self.write_staging_dir(&bag);
        let ready = parent.join("ready");
        let backup = parent.join(".write-previous");
        let mut expected = files
            .into_iter()
            .map(|file| MirrorFile {
                path: file.path,
                size: file.size,
                sha256: file.sha256,
            })
            .collect::<Vec<_>>();
        expected.sort_by(|left, right| left.path.cmp(&right.path));

        let source_dir = if staging.is_dir() {
            staging.clone()
        } else if ready.is_dir() {
            ready.clone()
        } else {
            return Err(format!("no staged upload exists for '{bag}'"));
        };
        let actual = crate::storage::cache::mirror_manifest(&source_dir)
            .map_err(|error| format!("could not verify staged upload '{bag}': {error}"))?;
        if actual != expected {
            return Err(format!(
                "cannot commit '{bag}': staged files do not match the completed local bag"
            ));
        }
        crate::io::bag::discover_bag(&source_dir)
            .map_err(|error| format!("cannot commit invalid ROS bag '{bag}': {error}"))?;

        if source_dir == staging {
            if backup.exists() {
                std::fs::remove_dir_all(&backup)
                    .map_err(|error| format!("could not clear old '{bag}' backup: {error}"))?;
            }
            if ready.exists() {
                std::fs::rename(&ready, &backup)
                    .map_err(|error| format!("could not preserve old '{bag}': {error}"))?;
            }
            if let Err(error) = std::fs::rename(&staging, &ready) {
                if backup.exists() {
                    let _ = std::fs::rename(&backup, &ready);
                }
                return Err(format!("could not publish staged '{bag}': {error}"));
            }
        }

        // The served registry still stores its ordinary packed object. Build
        // it losslessly from the atomically installed native bag, then delegate
        // the final write to the backing registry driver.
        let packed = parent.join(".write-bundle.marina.tar.gz");
        let ready_for_pack = ready.clone();
        let packed_for_task = packed.clone();
        let packed_meta = tokio::task::spawn_blocking(move || {
            let source = crate::io::bag::discover_bag(&ready_for_pack)?;
            let mut progress = crate::ProgressReporter::silent();
            crate::io::pack::pack_bag_with_progress_and_options(
                &source,
                &packed_for_task,
                PackOptions {
                    transform: crate::io::mcap_transform::PushTransformOptions {
                        pointcloud_mode: PointCloudCompressionMode::Disabled,
                        pointcloud_precision_m: 0.001,
                        output_mcap_compression: McapChunkCompression::None,
                    },
                    archive_compression: ArchiveCompression::Gzip,
                    db3_vacuum: false,
                },
                &mut progress,
            )
        })
        .await
        .map_err(|error| format!("packing '{bag}' panicked: {error}"))?
        .map_err(|error| format!("could not pack '{bag}': {error}"))?;
        let packed_bytes = std::fs::read(&packed)
            .map_err(|error| format!("could not hash packed '{bag}': {error}"))?;
        let bundle_hash = Sha256::digest(&packed_bytes)
            .iter()
            .take(6)
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let pushed_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        self.driver
            .push(
                &self.registry,
                &bag,
                &packed,
                &PushMeta {
                    original_bytes: packed_meta.original_bytes,
                    packed_bytes: packed_meta.packed_bytes,
                    bundle_hash,
                    pointcloud: "disabled".to_string(),
                    mcap_compression: "none".to_string(),
                    pushed_at,
                },
            )
            .await
            .map_err(|error| format!("could not publish '{bag}' to backing registry: {error}"))?;
        let _ = std::fs::remove_file(&packed);
        if backup.exists() {
            let _ = std::fs::remove_dir_all(&backup);
        }
        Ok(Response::WriteCommitted)
    }

    /// Fetch the bundle from the backing registry and start streaming it.
    ///
    /// The bundle is materialised into a temporary file first, because the
    /// backing driver's contract is "download to this path" and because the
    /// client is told the exact size up front so it can verify what it got.
    async fn begin_pull(&self, bag: crate::model::bag_ref::BagRef) -> Result<Response, String> {
        let staging = tempfile::Builder::new()
            .prefix("marina-serve-")
            .suffix(".tar.gz")
            .tempfile()
            .map_err(|error| format!("could not stage the bundle: {error}"))?;
        let staging_path = staging.path().to_path_buf();

        let descriptor = self
            .driver
            .pull(&bag, &staging_path)
            .await
            .map_err(|error| format!("could not read '{bag}' from the served registry: {error}"))?;

        let bytes = std::fs::read(&staging_path)
            .map_err(|error| format!("could not read the staged bundle: {error}"))?;
        let packed_bytes = bytes.len() as u64;

        // A flow name unique to this request, so concurrent pulls of the same
        // dataset do not land on the same topic.
        let flow = format!(
            "marina_{}_{}",
            self.registry,
            uuid_like(&bag.to_string(), packed_bytes)
        );

        let node = Arc::clone(&self.node);
        let config = self.flow;
        let flow_name = flow.clone();
        let label = bag.to_string();
        tokio::spawn(async move {
            // Held until the send finishes so the file outlives the transfer.
            let _staging = staging;
            let mut sender = match FlowSender::open(&node, &flow_name, config).await {
                Ok(sender) => sender,
                Err(error) => {
                    log::error!("marina serve: could not open flow for '{label}': {error}");
                    return;
                }
            };
            let mut source = BytesSource::new(bytes);
            match sender.send_all(&mut source).await {
                Ok(sent) => log::info!("marina serve: sent {sent} bytes of '{label}'"),
                Err(error) => log::error!("marina serve: sending '{label}' failed: {error}"),
            }
        });

        Ok(Response::PullBegin {
            flow,
            packed_bytes,
            original_bytes: descriptor.original_bytes,
        })
    }
}

#[derive(Default)]
struct SweepSummary {
    entries: usize,
    bytes: u64,
}

/// Result of a manual streaming-cache cleanup.
pub struct CacheCleanSummary {
    pub entries: usize,
    pub bytes: u64,
}

/// Remove expired unpacked streaming datasets and abandoned restore archives.
pub fn clean_streaming_cache(max_age: Duration) -> Result<CacheCleanSummary> {
    let materialize_root = crate::storage::config::cache_dir()
        .context("could not locate the cache directory")?
        .join("serve");
    let plan = prepare_cache_sweep(
        &materialize_root,
        &std::env::temp_dir(),
        max_age,
        &HashMap::new(),
    )
    .context("could not prepare streaming cache cleanup")?;
    let summary = remove_sweep_paths(plan).context("could not clean streaming cache")?;
    Ok(CacheCleanSummary {
        entries: summary.entries,
        bytes: summary.bytes,
    })
}

#[derive(Default)]
struct SweepPlan {
    trees: Vec<std::path::PathBuf>,
    files: Vec<std::path::PathBuf>,
}

fn prepare_cache_sweep(
    materialize_root: &std::path::Path,
    temp_root: &std::path::Path,
    max_age: Duration,
    last_access: &HashMap<String, SystemTime>,
) -> std::io::Result<SweepPlan> {
    let now = SystemTime::now();
    let mut plan = SweepPlan::default();

    if materialize_root.is_dir() {
        let trash = materialize_root.join(".sweep-trash");
        if trash.is_dir() {
            for entry in std::fs::read_dir(&trash)? {
                let entry = entry?;
                if entry.file_type()?.is_dir() {
                    plan.trees.push(entry.path());
                }
            }
        }
        for entry in std::fs::read_dir(materialize_root)? {
            let entry = entry?;
            let file_type = entry.file_type()?;
            if !file_type.is_dir() || file_type.is_symlink() {
                continue;
            }

            let path = entry.path();
            let is_upload = path.join(".write-incoming").exists()
                || path.join(".push-incoming.bundle").exists()
                || path.join(".push-incoming.identity").exists();
            let is_stream_cache = path.join("ready").is_dir() || path.join(".incoming").is_dir();
            if is_upload || !is_stream_cache {
                continue;
            }

            let key = entry.file_name().to_string_lossy().into_owned();
            let timestamp = last_access
                .get(&key)
                .copied()
                .or_else(|| {
                    std::fs::metadata(path.join(".last-access"))
                        .ok()?
                        .modified()
                        .ok()
                })
                .or_else(|| entry.metadata().ok()?.modified().ok())
                .unwrap_or(SystemTime::UNIX_EPOCH);
            if !is_expired(now, timestamp, max_age) {
                continue;
            }

            std::fs::create_dir_all(&trash)?;
            let suffix = now
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos();
            let staged = trash.join(format!("{key}-{suffix}"));
            std::fs::rename(path, &staged)?;
            plan.trees.push(staged);
        }
    }

    if temp_root.is_dir() {
        for entry in std::fs::read_dir(temp_root)? {
            let entry = entry?;
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if !name.starts_with("marina-materialize-") || !name.ends_with(".tar.gz") {
                continue;
            }
            let metadata = entry.metadata()?;
            if !metadata.file_type().is_file()
                || !is_expired(
                    now,
                    metadata.modified().unwrap_or(SystemTime::UNIX_EPOCH),
                    max_age,
                )
            {
                continue;
            }
            plan.files.push(entry.path());
        }
    }

    Ok(plan)
}

fn remove_sweep_paths(plan: SweepPlan) -> std::io::Result<SweepSummary> {
    let mut summary = SweepSummary::default();
    for path in plan.trees {
        summary.bytes += directory_size(&path);
        std::fs::remove_dir_all(path)?;
        summary.entries += 1;
    }
    for path in plan.files {
        summary.bytes += std::fs::metadata(&path)
            .map(|metadata| metadata.len())
            .unwrap_or(0);
        std::fs::remove_file(path)?;
        summary.entries += 1;
    }
    Ok(summary)
}

#[cfg(test)]
fn sweep_cache_paths(
    materialize_root: &std::path::Path,
    temp_root: &std::path::Path,
    max_age: Duration,
    last_access: &HashMap<String, SystemTime>,
) -> std::io::Result<SweepSummary> {
    remove_sweep_paths(prepare_cache_sweep(
        materialize_root,
        temp_root,
        max_age,
        last_access,
    )?)
}

fn is_expired(now: SystemTime, timestamp: SystemTime, max_age: Duration) -> bool {
    now.duration_since(timestamp).unwrap_or_default() >= max_age
}

fn directory_size(root: &std::path::Path) -> u64 {
    walkdir::WalkDir::new(root)
        .follow_links(false)
        .into_iter()
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| entry.metadata().ok())
        .filter(|metadata| metadata.is_file())
        .map(|metadata| metadata.len())
        .sum()
}

fn format_bytes(bytes: u64) -> String {
    const GIB: u64 = 1024 * 1024 * 1024;
    const MIB: u64 = 1024 * 1024;
    if bytes >= GIB {
        format!("{:.1} GiB", bytes as f64 / GIB as f64)
    } else if bytes >= MIB {
        format!("{:.1} MiB", bytes as f64 / MIB as f64)
    } else {
        format!("{bytes} bytes")
    }
}

/// Resolve a client-supplied relative path inside `root`, refusing anything
/// that escapes it.
///
/// The path comes off the network, so `../../etc/passwd` has to be impossible.
/// Rejecting unsafe path components keeps every request inside `root` without
/// depending on the target file already existing.
fn safe_join(root: &std::path::Path, relative: &str) -> Result<std::path::PathBuf, String> {
    use std::path::Component;

    if relative.is_empty() {
        return Err("an empty path is not a file in this dataset".to_string());
    }
    let candidate = std::path::Path::new(relative);
    for component in candidate.components() {
        match component {
            Component::Normal(_) => {}
            Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(format!("'{relative}' is not a path inside the dataset"));
            }
        }
    }
    Ok(root.join(candidate))
}

fn list_file_sizes(root: &std::path::Path) -> anyhow::Result<Vec<WireFile>> {
    let mut files = Vec::new();
    for entry in walkdir::WalkDir::new(root).follow_links(false) {
        let entry = entry?;
        if entry.path() == root || entry.file_type().is_dir() {
            continue;
        }
        anyhow::ensure!(
            !entry.file_type().is_symlink(),
            "symbolic links are not supported in streamed uploads"
        );
        files.push(WireFile {
            path: entry
                .path()
                .strip_prefix(root)?
                .to_string_lossy()
                .replace('\\', "/"),
            size: entry.metadata()?.len(),
        });
    }
    files.sort_by(|left, right| left.path.cmp(&right.path));
    Ok(files)
}

/// A short, collision-resistant-enough token for a flow name.
///
/// Not a real UUID: this only has to distinguish concurrent transfers within one
/// server, so the dataset name, its size, and the clock are plenty. Avoids
/// pulling in a uuid dependency for a naming detail.
fn uuid_like(seed: &str, size: u64) -> String {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    seed.hash(&mut hasher);
    size.hash(&mut hasher);
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .hash(&mut hasher);
    format!("{:016x}", hasher.finish())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flow_names_differ_between_concurrent_transfers() {
        let first = uuid_like("run:v1", 1024);
        let second = uuid_like("run:v1", 1024);
        assert_ne!(
            first, second,
            "two pulls of the same dataset must not share a flow topic"
        );
    }

    #[test]
    fn a_path_cannot_escape_the_dataset() {
        let root = std::path::Path::new("/srv/data");
        assert!(safe_join(root, "../../etc/passwd").is_err());
        assert!(safe_join(root, "nested/../../../etc/passwd").is_err());
        assert!(safe_join(root, "/etc/passwd").is_err());
        assert!(safe_join(root, "").is_err());
    }

    #[test]
    fn an_ordinary_path_resolves_inside_the_dataset() {
        let root = std::path::Path::new("/srv/data");
        assert_eq!(
            safe_join(root, "nested/run_0.mcap").unwrap(),
            root.join("nested/run_0.mcap")
        );
    }

    #[test]
    fn serve_options_default_to_this_machine_only() {
        let options = ServeOptions::new("team");
        assert!(
            options.local_only,
            "Minot has no authentication, so the safe default is loopback"
        );
        assert!(
            !options.allow_write,
            "remote writes must be explicitly enabled"
        );
        assert_eq!(options.cache_max_age, Duration::from_secs(96 * 60 * 60));
    }

    #[test]
    fn cache_sweep_removes_expired_stream_data_and_keeps_uploads() {
        let root = tempfile::tempdir().unwrap();
        let materialized = root.path().join("serve");
        let temp = root.path().join("tmp");
        std::fs::create_dir_all(materialized.join("expired/ready")).unwrap();
        std::fs::write(materialized.join("expired/ready/data.mcap"), b"bag").unwrap();
        std::fs::create_dir_all(materialized.join("recent/ready")).unwrap();
        std::fs::write(materialized.join("recent/ready/data.mcap"), b"bag").unwrap();
        std::fs::create_dir_all(materialized.join("upload/.write-incoming")).unwrap();
        std::fs::create_dir_all(&temp).unwrap();

        let last_access = HashMap::from([
            ("expired".to_string(), SystemTime::UNIX_EPOCH),
            ("recent".to_string(), SystemTime::now()),
            ("upload".to_string(), SystemTime::UNIX_EPOCH),
        ]);
        let summary = sweep_cache_paths(
            &materialized,
            &temp,
            Duration::from_secs(60 * 60),
            &last_access,
        )
        .unwrap();

        assert_eq!(summary.entries, 1);
        assert!(!materialized.join("expired").exists());
        assert!(materialized.join("recent").exists());
        assert!(materialized.join("upload").exists());
    }

    #[test]
    fn cache_sweep_removes_abandoned_materialization_archives() {
        let root = tempfile::tempdir().unwrap();
        let materialized = root.path().join("serve");
        let temp = root.path().join("tmp");
        std::fs::create_dir_all(&materialized).unwrap();
        std::fs::create_dir_all(&temp).unwrap();
        let abandoned = temp.join("marina-materialize-abandoned.tar.gz");
        let unrelated = temp.join("another-program.tar.gz");
        std::fs::write(&abandoned, b"archive").unwrap();
        std::fs::write(&unrelated, b"archive").unwrap();

        let summary =
            sweep_cache_paths(&materialized, &temp, Duration::ZERO, &HashMap::new()).unwrap();

        assert_eq!(summary.entries, 1);
        assert!(!abandoned.exists());
        assert!(unrelated.exists());
    }
}
