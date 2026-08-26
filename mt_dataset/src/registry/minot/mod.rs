//! A registry reached over a Minot network.
//!
//! `minot://` is deliberately not a new kind of registry — it is a *transport*
//! in front of an existing one. A `marina serve` process on the far side holds
//! a folder or SSH registry and answers on its behalf, so this driver
//! implements [`RegistryDriver`] with the same semantics as the registry behind
//! it.
//!
//! Making it boring is the point: the streaming work that follows depends on
//! this transport, and it is much easier to trust once `marina pull` over
//! `minot://` behaves exactly like `marina pull` over SSH.

pub mod block_store;
pub mod protocol;
pub mod remote_file;
pub mod server;

use std::any::Any;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use async_trait::async_trait;
use mt_flow::{ChunkSink, FlowConfig, FlowReceiver};
use mt_pubsub::{CoordMode, Node, NodeConfig, Qos};
use mt_service::ServiceClient;
use tokio::sync::OnceCell;

use crate::model::bag_ref::BagRef;
use crate::progress::ProgressReporter;
use crate::registry::driver::{
    BagInfo, PushMeta, RegistryDriver, RemoteDescriptor, StreamingDriver,
};
use crate::registry::ssh::{SshRegistry, SshTunnel};
use block_store::CacheMode;
use protocol::{
    PROTOCOL_VERSION, Request, Response, WireBagRef, WireFile, WireManifestFile, WirePushMeta,
    service_topic,
};
use remote_file::{NetworkFetcher, RangeFetcher, RemoteFile};

/// How long to wait for a control reply. Bulk transfer has its own, longer,
/// budget — this only covers a question the server should answer immediately.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(30);

/// Port a loopback-bound `marina serve` is reached on through a tunnel. Matches
/// Minot's fixed local coordinator endpoint.
const DEFAULT_COORDINATOR_PORT: u16 = 7447;

/// How the Minot network on the far side is reached.
enum Reach {
    /// Already reachable on this machine — a loopback coordinator, or a tunnel
    /// the user set up themselves.
    Local,
    /// A coordinator addressed directly. No authentication: only for a trusted
    /// network.
    Direct(String),
    /// Tunnelled over SSH, using an existing registry's credentials. This is
    /// the supported way to reach another machine, because `marina serve` binds
    /// loopback and the SSH connection is the security boundary.
    Ssh {
        ssh: Box<SshRegistry>,
        remote_port: u16,
    },
}

pub struct MinotRegistry {
    pub name: String,
    /// Registry name on the far side, from the URI.
    remote_registry: String,
    reach: Reach,
    flow: FlowConfig,
    /// Built on first use. Joining a Minot network is far too expensive to do
    /// while merely listing configured registries, and `marina` constructs every
    /// driver up front.
    session: OnceCell<Arc<Session>>,
}

struct Session {
    node: Arc<Node>,
    client: ServiceClient<Request, Response>,
    /// Held for the session's lifetime: dropping it closes the forward.
    _tunnel: Option<SshTunnel>,
}

impl Reach {
    fn describe(&self) -> String {
        match self {
            Reach::Local => "local".to_string(),
            Reach::Direct(address) => format!("direct({address})"),
            Reach::Ssh { remote_port, .. } => format!("ssh(->127.0.0.1:{remote_port})"),
        }
    }
}

impl std::fmt::Debug for MinotRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MinotRegistry")
            .field("name", &self.name)
            .field("remote_registry", &self.remote_registry)
            .field("reach", &self.reach.describe())
            .finish_non_exhaustive()
    }
}

impl MinotRegistry {
    /// Registry name requested from the remote `marina serve` process.
    pub fn remote_registry_name(&self) -> &str {
        &self.remote_registry
    }

    /// Open or resume a local-first dataset upload.
    pub async fn begin_write<'a>(&'a self, bag: &BagRef) -> Result<WriteSession<'a>> {
        let bag = bag.without_attachment();
        let response = self
            .request(Request::BeginWrite {
                bag: WireBagRef::from(&bag),
            })
            .await?;
        let Response::WriteStatus { files } = response else {
            return Err(anyhow!("unexpected begin-write response: {response:?}"));
        };
        Ok(WriteSession {
            registry: self,
            bag,
            offsets: files
                .into_iter()
                .map(|file| (file.path, file.size))
                .collect(),
        })
    }

    /// Parse a `minot://` URI.
    ///
    /// Three forms, in increasing order of how much they set up for you:
    ///
    /// - `minot://<registry>` — the server is already reachable on this machine
    ///   (both ends local, or a tunnel you opened yourself).
    /// - `minot://<host>:<port>/<registry>` — address a coordinator directly.
    ///   Use this on a trusted network. This transport has no authentication.
    /// - `minot+ssh://[user@]<host>[:port]/<registry>` — open an SSH tunnel and
    ///   reach a loopback-bound `marina serve` through it. This is the
    ///   supported way to reach another machine.
    pub fn from_uri(name: &str, uri: &str) -> Result<Self> {
        Self::from_uri_with_auth(name, uri, None, None, None)
    }

    /// As [`MinotRegistry::from_uri`], with the SSH settings a registry config
    /// carries. Ignored by the non-SSH forms.
    pub fn from_uri_with_auth(
        name: &str,
        uri: &str,
        auth_env: Option<String>,
        proxy_jump: Option<String>,
        ssh_transport: Option<String>,
    ) -> Result<Self> {
        if let Some(rest) = uri.strip_prefix("minot+ssh://") {
            let (target, registry) =
                rest.trim_end_matches('/').rsplit_once('/').ok_or_else(|| {
                    anyhow!(
                        "minot+ssh URI must name a registry, e.g. \
                     minot+ssh://user@host/team (got '{uri}')"
                    )
                })?;
            if target.is_empty() || registry.is_empty() {
                return Err(anyhow!(
                    "minot+ssh URI must be minot+ssh://[user@]host[:port]/<registry> (got '{uri}')"
                ));
            }
            // The SSH registry is used purely as a connection: its root path is
            // never read, because everything about the dataset comes from the
            // marina server on the far side.
            let ssh = SshRegistry::from_uri(
                &format!("{name}-tunnel"),
                &format!("ssh://{target}/"),
                auth_env,
                proxy_jump,
                ssh_transport,
            )
            .with_context(|| format!("could not read the ssh target in '{uri}'"))?;
            return Ok(Self {
                name: name.to_string(),
                remote_registry: registry.to_string(),
                reach: Reach::Ssh {
                    ssh: Box::new(ssh),
                    remote_port: DEFAULT_COORDINATOR_PORT,
                },
                flow: FlowConfig::default(),
                session: OnceCell::new(),
            });
        }

        let rest = uri.strip_prefix("minot://").ok_or_else(|| {
            anyhow!("minot registry URI must start with minot:// or minot+ssh://")
        })?;
        let rest = rest.trim_end_matches('/');
        if rest.is_empty() {
            return Err(anyhow!(
                "minot registry URI must name a registry, e.g. minot://team or minot://host:7447/team"
            ));
        }

        let (reach, remote_registry) = match rest.split_once('/') {
            Some((host, registry)) if !registry.is_empty() => {
                (Reach::Direct(format!("tcp/{host}")), registry.to_string())
            }
            // `minot://host:7447/` trims to a single segment, so it is not
            // distinguishable here from `minot://team/` by shape alone. The
            // check below catches it by content instead.
            _ => (Reach::Local, rest.to_string()),
        };

        // A colon identifies a host URI that needs a registry path.
        if remote_registry.contains(':') {
            return Err(anyhow!(
                "minot registry URI '{uri}' is missing a registry name. \
                 Use minot://<host>:<port>/<registry>, minot+ssh://<host>/<registry>, \
                 or minot://<registry> when reaching the server through a tunnel."
            ));
        }

        Ok(Self {
            name: name.to_string(),
            remote_registry,
            reach,
            flow: FlowConfig::default(),
            session: OnceCell::new(),
        })
    }

    async fn session(&self) -> Result<&Arc<Session>> {
        self.session
            .get_or_try_init(|| async {
                // Established before the node, because the node's very first
                // action is to reach the coordinator through it.
                let tunnel = match &self.reach {
                    Reach::Local => {
                        mt_sea::network::set_local_only(true);
                        None
                    }
                    Reach::Direct(address) => {
                        // SAFETY: set before any Minot session exists in this
                        // process. mt_sea reads it when opening a session.
                        unsafe { std::env::set_var("MINOT_COORD_ADDR", address) };
                        None
                    }
                    Reach::Ssh { ssh, remote_port } => {
                        let tunnel = ssh.open_local_forward(*remote_port).await.context(
                            "could not open an ssh tunnel to the marina server. Check the \
                             host and credentials, and that `marina serve` is running there.",
                        )?;
                        // The forward makes the remote server look local, which
                        // is exactly the shape the local form already handles.
                        unsafe {
                            std::env::set_var(
                                "MINOT_COORD_ADDR",
                                format!("tcp/127.0.0.1:{}", tunnel.local_port()),
                            )
                        };
                        Some(tunnel)
                    }
                };

                let node = Arc::new(
                    Node::create(
                        NodeConfig::new(format!("marina_client_{}", std::process::id()))
                            .mode(Qos::TryReliable)
                            .coord_mode(CoordMode::External)
                            .wan(),
                    )
                    .await
                    .context(
                        "could not join the Minot network. Is `marina serve` running, and \
                         is the tunnel to it up?",
                    )?,
                );

                let client = ServiceClient::<Request, Response>::new(
                    Arc::clone(&node),
                    service_topic(&self.remote_registry),
                )
                .await
                .context("could not open a request channel to the server")?;

                let session = Session {
                    node,
                    client,
                    _tunnel: tunnel,
                };
                // Check the version during the handshake so later requests use
                // compatible wire types.
                let response = session
                    .client
                    .request(
                        Request::Hello {
                            major: PROTOCOL_VERSION.0,
                            minor: PROTOCOL_VERSION.1,
                        },
                        Some(REQUEST_TIMEOUT),
                    )
                    .await
                    .map_err(|error| anyhow!("handshake with the marina server failed: {error}"))?;
                match response {
                    Response::Hello { major, minor, .. } => {
                        log::debug!(
                            "connected to a marina server speaking protocol {major}.{minor}"
                        );
                    }
                    other => return Err(anyhow!("unexpected handshake response: {other:?}")),
                }
                Ok::<_, anyhow::Error>(Arc::new(session))
            })
            .await
    }

    /// Ask what a dataset contains, and whether it can be read by range.
    ///
    /// The server materialises the dataset to answer, so the first call for a
    /// large dataset can take as long as a pull would.
    pub async fn stat(&self, bag: &BagRef) -> Result<StatResult> {
        let bag = bag.without_attachment();
        let mut last_status = std::time::Instant::now() - Duration::from_secs(30);
        let mut reconnecting = false;
        let response = loop {
            let response = match self
                .request_without_deadline(Request::Stat {
                    bag: WireBagRef::from(&bag),
                })
                .await
            {
                Ok(response) => {
                    if reconnecting {
                        log::info!("connection restored while waiting for '{bag}'");
                        reconnecting = false;
                    }
                    response
                }
                Err(error) if is_transient_service_error(&error) => {
                    if !reconnecting {
                        log::warn!(
                            "connection interrupted while waiting for '{bag}'. Reconnecting: {error}"
                        );
                        reconnecting = true;
                    }
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    continue;
                }
                Err(error) => return Err(error),
            };
            match response {
                Response::Materializing { message } => {
                    if last_status.elapsed() >= Duration::from_secs(5) {
                        log::info!("{message}. Waiting");
                        last_status = std::time::Instant::now();
                    }
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
                ready => break ready,
            }
        };
        let Response::Stat {
            files,
            streamable,
            reason,
        } = response
        else {
            return Err(anyhow!(
                "unexpected response to a stat request: {response:?}"
            ));
        };

        if !streamable {
            return Ok(StatResult::NotStreamable {
                reason: reason
                    .unwrap_or_else(|| "this dataset cannot be read by byte range".to_string()),
            });
        }

        let session = self.session().await?;
        let fetcher = self.range_fetcher(session, &bag)?;
        let cache = self.default_cache(&bag).await;
        Ok(StatResult::Streamable(Box::new(RemoteDataset {
            bag,
            files,
            fetcher,
            cache,
            _session: Arc::clone(session),
        })))
    }

    /// Open a dataset for reading without downloading it.
    ///
    /// Requires a streamable dataset. The caller controls pull fallback.
    pub async fn open_dataset(&self, bag: &BagRef) -> Result<RemoteDataset> {
        match self.stat(bag).await? {
            StatResult::Streamable(dataset) => Ok(*dataset),
            StatResult::NotStreamable { reason } => {
                Err(anyhow!("'{bag}' cannot be streamed: {reason}"))
            }
        }
    }

    /// Where streamed blocks are kept, and under what identity.
    ///
    /// Keyed by the dataset's bundle hash when the server knows one, so a cache
    /// written for one version of a dataset is never served for another. With
    /// no hash available the size still guards it, and a mismatch simply
    /// discards and refetches.
    async fn default_cache(&self, bag: &BagRef) -> CacheMode {
        let validity = self
            .bag_info(bag)
            .await
            .ok()
            .flatten()
            .and_then(|info| info.bundle_hash)
            .unwrap_or_else(|| "unknown".to_string());
        match crate::storage::config::cache_dir() {
            Ok(root) => CacheMode::Disk {
                root: root.join("stream").join(bag.cache_key()),
                validity,
            },
            Err(error) => {
                log::warn!(
                    "no cache directory available, streaming '{bag}' without keeping \
                     anything: {error}"
                );
                CacheMode::Ephemeral
            }
        }
    }

    /// Build the bridge that lets a synchronous reader issue range requests.
    fn range_fetcher(&self, session: &Arc<Session>, bag: &BagRef) -> Result<Arc<dyn RangeFetcher>> {
        let handle = tokio::runtime::Handle::try_current().map_err(|_| {
            anyhow!("opening a remote dataset requires a tokio runtime to be running")
        })?;
        let session = Arc::clone(session);
        let wire_bag = WireBagRef::from(bag);
        Ok(Arc::new(NetworkFetcher::spawn(
            handle,
            move |path, offset, len| {
                let session = Arc::clone(&session);
                let bag = wire_bag.clone();
                async move {
                    let mut reconnecting = false;
                    loop {
                        let shutdown = session.node.shutdown_token();
                        let response = tokio::select! {
                            _ = shutdown.cancelled() => {
                                return Err(anyhow!("range read stopped while the client was shutting down"));
                            }
                            response = session.client.request(
                                Request::ReadRange {
                                    bag: bag.clone(),
                                    path: path.clone(),
                                    offset,
                                    len,
                                },
                                Some(REQUEST_TIMEOUT),
                            ) => response,
                        };

                        match response {
                            Ok(Response::ReadRange { data }) => {
                                if reconnecting {
                                    log::info!(
                                        "connection restored while streaming '{}' at byte {offset}",
                                        path
                                    );
                                }
                                return Ok(data);
                            }
                            Ok(other) => {
                                return Err(anyhow!(
                                    "unexpected response to a range read: {other:?}"
                                ));
                            }
                            Err(error) => {
                                let error = anyhow!(error);
                                if !is_transient_service_error(&error) {
                                    return Err(error);
                                }
                                if !reconnecting {
                                    log::warn!(
                                        "connection interrupted while streaming '{}' at byte {offset}. Retrying: {error}",
                                        path
                                    );
                                    reconnecting = true;
                                }
                            }
                        }

                        let shutdown = session.node.shutdown_token();
                        tokio::select! {
                            _ = shutdown.cancelled() => {
                                return Err(anyhow!("range read stopped while the client was shutting down"));
                            }
                            _ = tokio::time::sleep(Duration::from_secs(1)) => {}
                        }
                    }
                }
            },
        )))
    }

    async fn request(&self, request: Request) -> Result<Response> {
        let session = self.session().await?;
        session
            .client
            .request(request, Some(REQUEST_TIMEOUT))
            .await
            .map_err(|error| anyhow!("{error}"))
    }

    /// Await a disk-bound server operation until it completes or the transport
    /// reports that the peer disappeared. Packing or publishing a large bag is
    /// not a control-plane timeout and has no honest fixed duration.
    async fn request_without_deadline(&self, request: Request) -> Result<Response> {
        let session = self.session().await?;
        session
            .client
            .request(request, None)
            .await
            .map_err(|error| anyhow!("{error}"))
    }
}

fn is_transient_service_error(error: &anyhow::Error) -> bool {
    let message = error.to_string();
    [
        "Error sending request:",
        "Timeout reached while sending request",
        "Timeout reached while waiting for response",
        "Response dispatcher stopped",
        "None response from ServiceServer",
    ]
    .iter()
    .any(|prefix| message.starts_with(prefix))
}

/// A dataset on the far side, opened for reading without downloading it.
///
/// A resumable upload whose durable watermarks live on the server.
///
/// The source directory remains the authoritative copy. Calling
/// [`WriteSession::sync_snapshot`] repeatedly tails files that are still
/// growing. [`WriteSession::commit`] is valid after the recorder has
/// closed the bag and written `metadata.yaml`.
pub struct WriteSession<'a> {
    registry: &'a MinotRegistry,
    bag: BagRef,
    offsets: std::collections::HashMap<String, u64>,
}

impl WriteSession<'_> {
    /// Upload every byte currently visible in `root`, resuming from server
    /// watermarks. Files may continue growing while this runs.
    pub async fn sync_snapshot(&mut self, root: &Path) -> Result<u64> {
        use std::io::{Read, Seek, SeekFrom};

        const UPLOAD_CHUNK_BYTES: usize = 1024 * 1024;
        let mut entries = Vec::new();
        for entry in walkdir::WalkDir::new(root).follow_links(false) {
            let entry = entry?;
            if entry.path() == root || entry.file_type().is_dir() {
                continue;
            }
            if entry.file_type().is_symlink() {
                return Err(anyhow!(
                    "symbolic links are not supported in streamed uploads: {}",
                    entry.path().display()
                ));
            }
            let relative = entry
                .path()
                .strip_prefix(root)?
                .to_string_lossy()
                .replace('\\', "/");
            entries.push((
                relative,
                entry.path().to_path_buf(),
                entry.metadata()?.len(),
            ));
        }
        entries.sort_by(|left, right| left.0.cmp(&right.0));

        let mut uploaded = 0u64;
        for (relative, path, snapshot_size) in entries {
            let mut offset = self.offsets.get(&relative).copied().unwrap_or(0);
            if offset > snapshot_size {
                return Err(anyhow!(
                    "remote staging for '{}' is {} bytes while the local file is {} bytes. Clear the interrupted upload before replacing this file",
                    relative,
                    offset,
                    snapshot_size
                ));
            }
            let mut file = std::fs::File::open(&path)
                .with_context(|| format!("could not open upload source {}", path.display()))?;
            while offset < snapshot_size {
                let len = (snapshot_size - offset).min(UPLOAD_CHUNK_BYTES as u64) as usize;
                let mut data = vec![0u8; len];
                file.seek(SeekFrom::Start(offset))?;
                file.read_exact(&mut data)?;
                let response = self
                    .registry
                    .request(Request::WriteRange {
                        bag: WireBagRef::from(&self.bag),
                        path: relative.clone(),
                        offset,
                        data,
                    })
                    .await
                    .with_context(|| format!("uploading '{}' at byte {offset}", relative))?;
                let Response::WriteAck { next_offset } = response else {
                    return Err(anyhow!("unexpected write response: {response:?}"));
                };
                if next_offset == offset {
                    return Err(anyhow!(
                        "server made no progress uploading '{}' at byte {offset}",
                        relative
                    ));
                }
                uploaded += next_offset.saturating_sub(offset);
                offset = next_offset;
                self.offsets.insert(relative.clone(), offset);
            }
        }
        Ok(uploaded)
    }

    /// Verify and atomically publish a closed local bag.
    pub async fn commit(mut self, root: &Path) -> Result<()> {
        self.sync_snapshot(root).await?;
        let files = crate::storage::cache::mirror_manifest(root)?
            .into_iter()
            .map(|file| WireManifestFile {
                path: file.path,
                size: file.size,
                sha256: file.sha256,
            })
            .collect();
        let response = self
            .registry
            .request_without_deadline(Request::CommitWrite {
                bag: WireBagRef::from(&self.bag),
                files,
            })
            .await?;
        match response {
            Response::WriteCommitted => Ok(()),
            other => Err(anyhow!("unexpected commit response: {other:?}")),
        }
    }
}

/// Holds the file list and hands out [`RemoteFile`]s. Everything it returns
/// implements `Read + Seek`, which is all a sans-io bag reader needs.
pub struct RemoteDataset {
    bag: BagRef,
    files: Vec<WireFile>,
    fetcher: Arc<dyn RangeFetcher>,
    cache: CacheMode,
    /// Kept alive so the transport outlives every file handed out.
    _session: Arc<Session>,
}

impl RemoteDataset {
    /// Every file in the dataset, with its size.
    pub fn files(&self) -> &[WireFile] {
        &self.files
    }

    pub fn bag(&self) -> &BagRef {
        &self.bag
    }

    /// Total size of the dataset as it would be on disk.
    pub fn total_bytes(&self) -> u64 {
        self.files.iter().map(|file| file.size).sum()
    }

    /// Choose what happens to the blocks this dataset's files fetch.
    ///
    /// The default is [`CacheMode::Disk`]: reading warms a local copy, so a
    /// second read is free and a complete read has effectively pulled the
    /// dataset. Switch to [`CacheMode::Ephemeral`] for a read that leaves
    /// nothing behind — the file is never touched on disk and memory stays
    /// bounded regardless of the dataset's size.
    pub fn with_cache(mut self, cache: CacheMode) -> Self {
        self.cache = cache;
        self
    }

    /// Read this dataset without keeping any of it.
    ///
    /// Shorthand for [`RemoteDataset::with_cache`] with
    /// [`CacheMode::Ephemeral`].
    pub fn online_only(self) -> Self {
        self.with_cache(CacheMode::Ephemeral)
    }

    /// Whether reads of this dataset leave a local copy behind.
    pub fn persists(&self) -> bool {
        matches!(self.cache, CacheMode::Disk { .. })
    }

    /// Open one file by its path relative to the dataset root.
    pub fn open(&self, path: &str) -> Result<RemoteFile> {
        let file = self
            .files
            .iter()
            .find(|candidate| candidate.path == path)
            .ok_or_else(|| {
                anyhow!(
                    "'{path}' is not in dataset '{}'. It contains: {}",
                    self.bag,
                    self.files
                        .iter()
                        .map(|f| f.path.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            })?;
        Ok(RemoteFile::with_cache(
            Arc::clone(&self.fetcher),
            file.path.clone(),
            file.size,
            &self.cache,
        ))
    }

    /// How much of this dataset is already held locally, 0.0 to 1.0.
    ///
    /// Always 0.0 in ephemeral mode, where nothing is held by design.
    pub fn cached_fraction(&self) -> f32 {
        let CacheMode::Disk { root, validity } = &self.cache else {
            return 0.0;
        };
        let total: u64 = self.files.iter().map(|file| file.size).sum();
        if total == 0 {
            return 1.0;
        }
        let held: u64 = self
            .files
            .iter()
            .filter(|file| {
                block_store::completed_file(
                    root,
                    &file.path,
                    file.size,
                    remote_file::DEFAULT_BLOCK_BYTES,
                    validity,
                )
                .is_some()
            })
            .map(|file| file.size)
            .sum();
        held as f32 / total as f32
    }

    /// Read every byte of the dataset, then install it as an ordinary local one.
    ///
    /// This is the point the whole design turns on: **streaming and pulling are
    /// the same operation in different orders.** A streamed read fills the
    /// block cache. Once every block of every file is present, the sparse files
    /// *are* the dataset, so they are moved into `ready/` and registered in the
    /// catalog. From then on `marina resolve` returns a plain path and nothing
    /// downstream knows or cares that it arrived by streaming.
    ///
    /// Unlike a pull, this resumes: blocks fetched by an earlier interrupted
    /// read or by ordinary use are not fetched again.
    ///
    /// Refuses in ephemeral mode, where there is deliberately nothing to
    /// promote.
    pub fn materialize(&self, progress: &mut ProgressReporter<'_>) -> Result<PathBuf> {
        let CacheMode::Disk { root, validity } = &self.cache else {
            return Err(anyhow!(
                "'{}' is being read in online-only mode, which keeps nothing; \
                 use the default cache mode to materialise it",
                self.bag
            ));
        };

        for file in &self.files {
            if block_store::completed_file(
                root,
                &file.path,
                file.size,
                remote_file::DEFAULT_BLOCK_BYTES,
                validity,
            )
            .is_some()
            {
                progress.emit("stream", format!("{} already complete", file.path));
                continue;
            }
            progress.emit("stream", format!("fetching {}", file.path));
            let mut remote = self.open(&file.path)?;
            // Read it through: the bytes are wanted only for their effect on
            // the cache. Ephemeral mode discards completed blocks as they arrive.
            std::io::copy(&mut remote, &mut std::io::sink())
                .with_context(|| format!("failed streaming '{}'", file.path))?;
            if !remote.is_complete() {
                return Err(anyhow!(
                    "streamed all of '{}' but its cache is still incomplete",
                    file.path
                ));
            }
        }

        self.install(root, validity, progress)
    }

    /// Move the completed sparse files into `ready/` and register the dataset.
    fn install(
        &self,
        root: &std::path::Path,
        validity: &str,
        progress: &mut ProgressReporter<'_>,
    ) -> Result<PathBuf> {
        let cache_dir = crate::storage::cache::bag_cache_dir(&self.bag)?;
        let ready = cache_dir.join("ready");
        // Assembled beside the destination and renamed, so an interrupted
        // install never leaves a half-built tree that looks ready.
        let staging = cache_dir.join(".stream-incoming");
        if staging.exists() {
            std::fs::remove_dir_all(&staging)?;
        }
        std::fs::create_dir_all(&staging)?;

        for file in &self.files {
            let complete = block_store::completed_file(
                root,
                &file.path,
                file.size,
                remote_file::DEFAULT_BLOCK_BYTES,
                validity,
            )
            .ok_or_else(|| {
                anyhow!(
                    "cannot install '{}': '{}' is not fully cached",
                    self.bag,
                    file.path
                )
            })?;
            let target = staging.join(&file.path);
            if let Some(parent) = target.parent() {
                std::fs::create_dir_all(parent)?;
            }
            // A rename keeps the bytes where they are when the cache and the
            // dataset usually share a filesystem. The copy is
            // the fallback when they do not.
            if std::fs::rename(&complete, &target).is_err() {
                std::fs::copy(&complete, &target).with_context(|| {
                    format!("failed installing '{}' from the stream cache", file.path)
                })?;
                let _ = std::fs::remove_file(&complete);
            }
        }

        if ready.exists() {
            std::fs::remove_dir_all(&ready)?;
        }
        std::fs::rename(&staging, &ready)
            .with_context(|| format!("failed installing '{}'", self.bag))?;

        let mut catalog = crate::storage::cache::load_catalog()?;
        catalog.entries.insert(
            self.bag.to_string(),
            crate::storage::cache::CacheEntry {
                bag: self.bag.clone(),
                local_dir: ready.clone(),
                packed_bytes: 0,
                bundle_hash: Some(validity.to_string()),
            },
        );
        crate::storage::cache::save_catalog(&catalog)?;

        progress.emit("stream", format!("{} is now available locally", self.bag));
        Ok(ready)
    }

    /// Open the dataset's MCAP, when it has exactly one.
    ///
    /// The common case by far, and it saves every caller from rediscovering how
    /// to find it.
    pub fn open_mcap(&self) -> Result<RemoteFile> {
        let mcaps: Vec<&WireFile> = self
            .files
            .iter()
            .filter(|file| file.path.ends_with(".mcap"))
            .collect();
        match mcaps.as_slice() {
            [] => Err(anyhow!("dataset '{}' contains no .mcap file", self.bag)),
            [only] => self.open(&only.path),
            many => Err(anyhow!(
                "dataset '{}' contains {} MCAP files. Open one by name: {}",
                self.bag,
                many.len(),
                many.iter()
                    .map(|f| f.path.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )),
        }
    }
}

/// What a `Stat` said about a dataset.
pub enum StatResult {
    /// Readable by range. Open it as a [`RemoteDataset`].
    Streamable(Box<RemoteDataset>),
    /// Not readable by range — a sqlite3 bag. Pull it instead.
    NotStreamable { reason: String },
}

/// Writes a flow straight to disk at the offset the sender gave it.
///
/// A sparse write is exactly what a resumed transfer produces, so the file is
/// extended as needed to support resumed chunks in any order.
struct FileSink {
    file: std::fs::File,
}

impl ChunkSink for FileSink {
    fn write_at(&mut self, offset: u64, payload: &[u8]) -> anyhow::Result<()> {
        use std::io::{Seek, SeekFrom, Write};
        self.file.seek(SeekFrom::Start(offset))?;
        self.file.write_all(payload)?;
        Ok(())
    }
}

#[async_trait]
impl RegistryDriver for MinotRegistry {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_streaming(&self) -> Option<&dyn StreamingDriver> {
        Some(self)
    }

    async fn push(
        &self,
        _registry_name: &str,
        bag: &BagRef,
        packed_file: &Path,
        meta: &PushMeta,
    ) -> Result<()> {
        use std::io::{Read, Seek, SeekFrom};

        const UPLOAD_CHUNK_BYTES: usize = 1024 * 1024;
        let bag = bag.without_attachment();
        let actual_size = std::fs::metadata(packed_file)
            .with_context(|| format!("could not inspect {}", packed_file.display()))?
            .len();
        if actual_size != meta.packed_bytes {
            return Err(anyhow!(
                "packed bundle is {actual_size} bytes but its push metadata says {} bytes",
                meta.packed_bytes
            ));
        }

        let response = self
            .request(Request::BeginPush {
                bag: WireBagRef::from(&bag),
                packed_bytes: meta.packed_bytes,
                bundle_hash: meta.bundle_hash.clone(),
            })
            .await?;
        let Response::PushStatus { mut next_offset } = response else {
            return Err(anyhow!("unexpected begin-push response: {response:?}"));
        };
        if next_offset > actual_size {
            return Err(anyhow!(
                "remote staging for '{bag}' is {next_offset} bytes but the local bundle is only {actual_size} bytes"
            ));
        }

        let mut file = std::fs::File::open(packed_file)
            .with_context(|| format!("could not open {}", packed_file.display()))?;
        while next_offset < actual_size {
            let offset = next_offset;
            let len = (actual_size - offset).min(UPLOAD_CHUNK_BYTES as u64) as usize;
            let mut data = vec![0u8; len];
            file.seek(SeekFrom::Start(offset))?;
            file.read_exact(&mut data)?;
            let response = self
                .request(Request::PushRange {
                    bag: WireBagRef::from(&bag),
                    offset,
                    data,
                })
                .await
                .with_context(|| format!("uploading packed '{bag}' at byte {offset}"))?;
            let Response::PushAck {
                next_offset: acknowledged,
            } = response
            else {
                return Err(anyhow!("unexpected packed-push response: {response:?}"));
            };
            if acknowledged == offset {
                return Err(anyhow!(
                    "server made no progress uploading packed '{bag}' at byte {offset}"
                ));
            }
            if acknowledged > actual_size {
                return Err(anyhow!(
                    "server acknowledged byte {acknowledged} beyond the {actual_size}-byte packed bundle"
                ));
            }
            next_offset = acknowledged;
        }

        match self
            .request_without_deadline(Request::CommitPush {
                bag: WireBagRef::from(&bag),
                meta: WirePushMeta::from(meta),
            })
            .await?
        {
            Response::PushCommitted => Ok(()),
            other => Err(anyhow!("unexpected push-commit response: {other:?}")),
        }
    }

    async fn pull(&self, bag: &BagRef, out_packed_file: &Path) -> Result<RemoteDescriptor> {
        let response = self
            .request(Request::PullBegin {
                bag: WireBagRef::from(bag),
            })
            .await?;
        let Response::PullBegin {
            flow,
            packed_bytes,
            original_bytes,
        } = response
        else {
            return Err(anyhow!(
                "unexpected response to a pull request: {response:?}"
            ));
        };

        if let Some(parent) = out_packed_file.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let file = std::fs::File::create(out_packed_file)
            .with_context(|| format!("failed to create {}", out_packed_file.display()))?;
        let mut sink = FileSink { file };

        let session = self.session().await?;
        let mut receiver = FlowReceiver::open(&session.node, &flow, self.flow)
            .await
            .context("could not open the transfer")?;
        let received = receiver
            .receive_all(&mut sink)
            .await
            .with_context(|| format!("transfer of '{bag}' failed"))?;

        // The server told us the size up front, so a stream that ended early
        // so a short transfer is reported before the bundle is installed.
        if received != packed_bytes {
            return Err(anyhow!(
                "incomplete transfer of '{bag}': received {received} of {packed_bytes} bytes"
            ));
        }

        Ok(RemoteDescriptor {
            registry_name: self.name.clone(),
            bag: bag.clone(),
            original_bytes,
            packed_bytes,
        })
    }

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>> {
        let response = self
            .request(Request::List {
                pattern: filter.to_string(),
            })
            .await?;
        match response {
            Response::List(bags) => Ok(bags.into_iter().map(Into::into).collect()),
            other => Err(anyhow!("unexpected response to a list request: {other:?}")),
        }
    }

    async fn remove(&self, bag: &BagRef) -> Result<()> {
        let response = self
            .request(Request::Remove {
                bag: WireBagRef::from(&bag.without_attachment()),
            })
            .await?;
        match response {
            Response::Removed => Ok(()),
            other => Err(anyhow!("unexpected remove response: {other:?}")),
        }
    }

    async fn bag_info(&self, bag: &BagRef) -> Result<Option<BagInfo>> {
        let response = self
            .request(Request::BagInfo {
                bag: WireBagRef::from(bag),
            })
            .await?;
        match response {
            Response::BagInfo(info) => Ok(info.map(Into::into)),
            other => Err(anyhow!(
                "unexpected response to a bag_info request: {other:?}"
            )),
        }
    }

    async fn check_connection(&self) -> Result<()> {
        // The handshake is the connection check.
        self.session().await.map(|_| ())
    }

    async fn check_write_access(&self) -> Result<()> {
        match self.request(Request::CheckWrite).await? {
            Response::WriteAllowed => Ok(()),
            other => Err(anyhow!("unexpected write-access response: {other:?}")),
        }
    }
}

#[async_trait]
impl StreamingDriver for MinotRegistry {
    async fn open_dataset(&self, bag: &BagRef) -> Result<RemoteDataset> {
        MinotRegistry::open_dataset(self, bag).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_registry_name_means_the_local_coordinator() {
        let registry = MinotRegistry::from_uri("team", "minot://team").unwrap();
        assert_eq!(registry.remote_registry, "team");
        assert!(
            matches!(registry.reach, Reach::Local),
            "with no host named, the server is already reachable on this machine"
        );
    }

    #[test]
    fn a_host_and_registry_are_split_apart() {
        let registry = MinotRegistry::from_uri("lab", "minot://192.0.2.4:7447/lab").unwrap();
        assert_eq!(registry.remote_registry, "lab");
        assert!(
            matches!(&registry.reach, Reach::Direct(address) if address == "tcp/192.0.2.4:7447"),
            "got {}",
            registry.reach.describe()
        );
    }

    #[test]
    fn a_trailing_slash_is_tolerated() {
        let registry = MinotRegistry::from_uri("team", "minot://team/").unwrap();
        assert_eq!(registry.remote_registry, "team");
    }

    #[test]
    fn a_uri_without_a_registry_is_rejected() {
        assert!(MinotRegistry::from_uri("x", "minot://").is_err());
        assert!(
            MinotRegistry::from_uri("x", "minot://host:7447/").is_err(),
            "naming a host but no registry is a typo, not a default"
        );
    }

    #[test]
    fn a_non_minot_uri_is_rejected() {
        assert!(MinotRegistry::from_uri("x", "ssh://host/path").is_err());
    }

    #[test]
    fn an_ssh_uri_tunnels_to_the_far_side() {
        let registry = MinotRegistry::from_uri("lab", "minot+ssh://user@host.example/lab").unwrap();
        assert_eq!(registry.remote_registry, "lab");
        assert!(
            matches!(registry.reach, Reach::Ssh { remote_port, .. }
                     if remote_port == DEFAULT_COORDINATOR_PORT),
            "an ssh URI must reach the far side's loopback coordinator, got {}",
            registry.reach.describe()
        );
    }

    #[test]
    fn an_ssh_uri_without_a_registry_is_rejected() {
        assert!(
            MinotRegistry::from_uri("x", "minot+ssh://user@host.example").is_err(),
            "a bare host names no registry to serve"
        );
        assert!(MinotRegistry::from_uri("x", "minot+ssh://").is_err());
    }

    #[test]
    fn the_three_uri_forms_are_distinguishable() {
        let local = MinotRegistry::from_uri("a", "minot://team").unwrap();
        let direct = MinotRegistry::from_uri("b", "minot://host:7447/team").unwrap();
        let tunnelled = MinotRegistry::from_uri("c", "minot+ssh://host/team").unwrap();
        assert!(matches!(local.reach, Reach::Local));
        assert!(matches!(direct.reach, Reach::Direct(_)));
        assert!(matches!(tunnelled.reach, Reach::Ssh { .. }));
        // All three name the same registry on the far side.
        for registry in [&local, &direct, &tunnelled] {
            assert_eq!(registry.remote_registry, "team");
        }
    }

    #[test]
    fn transient_service_failures_are_retryable() {
        for message in [
            "Error sending request: connection closed",
            "Timeout reached while sending request",
            "Timeout reached while waiting for response",
            "Response dispatcher stopped",
            "None response from ServiceServer",
        ] {
            assert!(is_transient_service_error(&anyhow!(message)));
        }
    }

    #[test]
    fn server_errors_are_returned_to_the_caller() {
        assert!(!is_transient_service_error(&anyhow!(
            "materialising 'run' failed: archive is corrupt"
        )));
    }
}
