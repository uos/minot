use std::collections::HashMap;
use std::fs;
use std::io::IsTerminal;
use std::path::Path;
use std::sync::Arc;
#[cfg(feature = "minot-registry")]
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use glob::Pattern;
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use russh::ChannelMsg;
use russh::client::{self, Config, Handle};
use russh::keys::PrivateKeyWithHashAlg;
use russh::keys::PublicKeyOrCertificate;
use russh_sftp::client::SftpSession;
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::process::Command;
use tokio::sync::Mutex;

#[cfg(feature = "minot-registry")]
const REMOTE_FORWARD_READY_TIMEOUT: Duration = Duration::from_secs(60);
#[cfg(feature = "minot-registry")]
const REMOTE_FORWARD_RETRY_INTERVAL: Duration = Duration::from_millis(500);

use crate::model::bag_ref::BagRef;
use crate::registry::driver::{BagInfo, PushMeta, RegistryDriver, RemoteDescriptor};
use crate::storage::cache::MirrorFile;

pub struct SshRegistry {
    pub name: String,
    endpoint: SshEndpoint,
    auth_env: Option<String>,
    proxy_jump: Option<SshEndpoint>,
    transport: SshTransport,
    pool: Arc<Mutex<Option<Arc<Handle<ClientHandler>>>>>,
}

impl std::fmt::Debug for SshRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SshRegistry")
            .field("name", &self.name)
            .field("endpoint", &self.endpoint)
            .field("proxy_jump", &self.proxy_jump)
            .field("transport", &self.transport)
            .finish_non_exhaustive()
    }
}

impl Clone for SshRegistry {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            endpoint: self.endpoint.clone(),
            auth_env: self.auth_env.clone(),
            proxy_jump: self.proxy_jump.clone(),
            transport: self.transport,
            pool: Arc::clone(&self.pool),
        }
    }
}

/// A live local-port forward. Dropping it tears the forward down.
///
/// Only built for `minot://` registries, which are feature-gated.
#[cfg(feature = "minot-registry")]
pub(crate) struct SshTunnel {
    local_port: u16,
    /// The accept loop, for the native transport.
    accept: Option<tokio::task::JoinHandle<()>>,
    /// The `ssh -L` process, for the OpenSSH transport.
    child: Option<tokio::process::Child>,
    /// Drains OpenSSH's stderr after startup so repeated reconnect diagnostics
    /// cannot fill its pipe and stall the forwarding process.
    stderr_drain: Option<tokio::task::JoinHandle<()>>,
}

#[cfg(feature = "minot-registry")]
impl SshTunnel {
    /// Loopback port on this machine that now reaches the far side.
    pub(crate) fn local_port(&self) -> u16 {
        self.local_port
    }
}

#[cfg(feature = "minot-registry")]
impl Drop for SshTunnel {
    fn drop(&mut self) {
        if let Some(accept) = self.accept.take() {
            accept.abort();
        }
        if let Some(mut child) = self.child.take() {
            let _ = child.start_kill();
        }
        if let Some(stderr_drain) = self.stderr_drain.take() {
            stderr_drain.abort();
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SshTransport {
    Native,
    OpenSsh,
}

#[derive(Debug, Clone, Copy)]
enum CopyDirection {
    Upload,
    Download,
}

#[derive(Debug, Clone)]
struct SshEndpoint {
    user_host: String,
    port: u16,
    root: String,
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

struct ClientHandler;

impl client::Handler for ClientHandler {
    type Error = anyhow::Error;

    async fn check_server_key(
        &mut self,
        _server_public_key: &PublicKeyOrCertificate,
    ) -> Result<bool, Self::Error> {
        Ok(true)
    }
}

impl SshRegistry {
    pub fn from_uri(
        name: &str,
        uri: &str,
        auth_env: Option<String>,
        proxy_jump: Option<String>,
        ssh_transport: Option<String>,
    ) -> Result<Self> {
        let endpoint = SshEndpoint::parse(uri)?;
        let proxy_jump = proxy_jump
            .or_else(|| std::env::var("MARINA_SSH_PROXY_JUMP").ok())
            .map(|jump| SshEndpoint::parse_jump(&jump))
            .transpose()?;
        let transport = parse_transport(
            ssh_transport
                .or_else(|| std::env::var("MARINA_SSH_TRANSPORT").ok())
                .as_deref(),
        )?;
        Ok(Self {
            name: name.to_string(),
            endpoint,
            auth_env,
            proxy_jump,
            transport,
            pool: Arc::new(Mutex::new(None)),
        })
    }

    fn object_dir(&self, bag: &BagRef) -> String {
        format!("{}/{}", self.endpoint.root, bag.object_path())
    }

    fn data_path(&self, bag: &BagRef) -> String {
        format!("{}/bundle.marina.tar.gz", self.object_dir(bag))
    }

    fn meta_path(&self, bag: &BagRef) -> String {
        format!("{}/metadata.json", self.object_dir(bag))
    }

    async fn connect_inner(&self) -> Result<Handle<ClientHandler>> {
        let config = Arc::new(Config::default());

        let mut handle = if let Some(proxy_jump) = &self.proxy_jump {
            let (_target_user, target_host) = split_user_host(&self.endpoint.user_host)?;
            let mut jump = connect_endpoint(config.clone(), proxy_jump)
                .await
                .with_context(|| {
                    format!(
                        "failed connecting to ssh proxy jump {}",
                        proxy_jump.display_host_port()
                    )
                })?;
            self.authenticate_endpoint(&mut jump, proxy_jump).await?;

            let channel = jump
                .channel_open_direct_tcpip(
                    target_host.clone(),
                    self.endpoint.port.into(),
                    "127.0.0.1",
                    0,
                )
                .await
                .with_context(|| {
                    format!(
                        "failed opening ssh tunnel via {} to {}:{}",
                        proxy_jump.display_host_port(),
                        target_host,
                        self.endpoint.port
                    )
                })?;

            client::connect_stream(config, channel.into_stream(), ClientHandler)
                .await
                .with_context(|| {
                    format!(
                        "failed connecting to ssh host {}:{} via {}",
                        target_host,
                        self.endpoint.port,
                        proxy_jump.display_host_port()
                    )
                })?
        } else {
            connect_endpoint(config, &self.endpoint)
                .await
                .with_context(|| {
                    let (_user, host) = split_user_host(&self.endpoint.user_host)
                        .unwrap_or_else(|_| (String::new(), self.endpoint.user_host.clone()));
                    format!(
                        "failed connecting to ssh host {}:{}",
                        host, self.endpoint.port
                    )
                })?
        };

        self.authenticate_endpoint(&mut handle, &self.endpoint)
            .await?;

        Ok(handle)
    }

    async fn authenticate_endpoint(
        &self,
        handle: &mut Handle<ClientHandler>,
        endpoint: &SshEndpoint,
    ) -> Result<()> {
        let (user, host) = split_user_host(&endpoint.user_host)?;

        let authed = match &self.auth_env {
            Some(var) => {
                let secret = std::env::var(var)
                    .with_context(|| format!("missing ssh auth env var '{}'", var))?;
                let secret_path = Path::new(&secret);
                if secret_path.exists() {
                    let passphrase_var = format!("{}_PASSPHRASE", var);
                    let passphrase = std::env::var(&passphrase_var).ok();
                    let key = russh::keys::load_secret_key(secret_path, passphrase.as_deref())
                        .with_context(|| {
                            format!("failed loading ssh key {}", secret_path.display())
                        })?;
                    handle
                        .authenticate_publickey(
                            &user,
                            PrivateKeyWithHashAlg::new(Arc::new(key), None),
                        )
                        .await
                        .with_context(|| {
                            format!(
                                "ssh key auth failed for user '{}' using key {}",
                                user,
                                secret_path.display()
                            )
                        })?
                        .success()
                } else {
                    handle
                        .authenticate_password(&user, &secret)
                        .await
                        .with_context(|| format!("ssh password auth failed for user '{}'", user))?
                        .success()
                }
            }
            None => {
                let mut authed = false;

                // Try ssh-agent
                if let Ok(sock) = std::env::var("SSH_AUTH_SOCK") {
                    if let Ok(mut agent) =
                        russh::keys::agent::client::AgentClient::connect_uds(&sock).await
                    {
                        if let Ok(identities) = agent.request_identities().await {
                            for identity in identities {
                                if handle
                                    .authenticate_publickey_with(
                                        &user,
                                        identity.public_key().into_owned(),
                                        None,
                                        &mut agent,
                                    )
                                    .await
                                    .map(|r| r.success())
                                    .unwrap_or(false)
                                {
                                    authed = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                // Try default key files
                if !authed {
                    if let Some(home) = dirs::home_dir() {
                        let ssh_dir = home.join(".ssh");
                        for key_name in ["id_ed25519", "id_rsa", "id_ecdsa", "id_dsa"] {
                            let key_path = ssh_dir.join(key_name);
                            if key_path.exists() {
                                if let Ok(key) = russh::keys::load_secret_key(&key_path, None) {
                                    if handle
                                        .authenticate_publickey(
                                            &user,
                                            PrivateKeyWithHashAlg::new(Arc::new(key), None),
                                        )
                                        .await
                                        .map(|r| r.success())
                                        .unwrap_or(false)
                                    {
                                        authed = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // Fall back to interactive password prompt
                if !authed {
                    let password =
                        rpassword::prompt_password(format!("Password for {}@{}: ", user, host))
                            .context("failed reading password")?;
                    handle
                        .authenticate_password(&user, &password)
                        .await
                        .with_context(|| format!("ssh password auth failed for user '{}'", user))?
                        .success()
                } else {
                    true
                }
            }
        };

        if !authed {
            return Err(anyhow!("ssh authentication failed"));
        }

        Ok(())
    }

    /// Forward a local port to `remote_port` on the far side's loopback.
    ///
    /// This is what makes `marina serve` reachable across machines. The server
    /// binds loopback only because Minot carries no authentication of its own,
    /// so the SSH connection *is* the security boundary — and the credentials
    /// are the ones already configured for this registry, so nothing new has to
    /// be set up.
    ///
    /// The returned [`SshTunnel`] keeps the forwarder alive. Dropping it stops
    /// accepting new connections.
    #[cfg(feature = "minot-registry")]
    pub(crate) async fn open_local_forward(&self, remote_port: u16) -> Result<SshTunnel> {
        if self.transport == SshTransport::OpenSsh {
            return self.open_local_forward_openssh(remote_port).await;
        }

        let handle = self.get_handle().await?;
        self.wait_for_remote_port(&handle, remote_port).await?;
        // Port 0: let the OS pick, so two marina processes on one machine do not
        // collide over a hardcoded choice.
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .context("failed to bind a local port for the ssh tunnel")?;
        let local_port = listener
            .local_addr()
            .context("failed to read the tunnel's local address")?
            .port();

        let host_label = self.endpoint.display_host_port();
        let accept = tokio::spawn(async move {
            loop {
                let Ok((mut local, _)) = listener.accept().await else {
                    return;
                };
                let handle = Arc::clone(&handle);
                let host_label = host_label.clone();
                tokio::spawn(async move {
                    let channel = match handle
                        .channel_open_direct_tcpip("127.0.0.1", remote_port.into(), "127.0.0.1", 0)
                        .await
                    {
                        Ok(channel) => channel,
                        Err(error) => {
                            log::error!(
                                "ssh tunnel to {host_label}: could not reach 127.0.0.1:{remote_port} \
                                 on the far side — is `marina serve` running there? ({error})"
                            );
                            return;
                        }
                    };
                    let mut remote = channel.into_stream();
                    // Errors here are ordinary connection lifecycle, not faults.
                    if let Err(error) = tokio::io::copy_bidirectional(&mut local, &mut remote).await
                    {
                        log::debug!("ssh tunnel connection ended: {error}");
                    }
                });
            }
        });

        log::info!(
            "ssh tunnel: 127.0.0.1:{local_port} -> {}:{remote_port}",
            self.endpoint.display_host_port()
        );
        Ok(SshTunnel {
            local_port,
            accept: Some(accept),
            child: None,
            stderr_drain: None,
        })
    }

    /// Do not advertise a forward until SSH has proved that its destination is
    /// accepting connections. Binding the local listener alone only proves the
    /// near side. This check avoids Zenoh reconnect noise during server startup.
    #[cfg(feature = "minot-registry")]
    async fn wait_for_remote_port(
        &self,
        handle: &Arc<Handle<ClientHandler>>,
        remote_port: u16,
    ) -> Result<()> {
        let started = Instant::now();
        let mut announced_wait = false;
        let mut last_error = None;

        while started.elapsed() < REMOTE_FORWARD_READY_TIMEOUT {
            match tokio::time::timeout(
                Duration::from_secs(3),
                handle.channel_open_direct_tcpip("127.0.0.1", remote_port.into(), "127.0.0.1", 0),
            )
            .await
            {
                Ok(Ok(channel)) => {
                    drop(channel);
                    return Ok(());
                }
                Ok(Err(error)) => last_error = Some(error.to_string()),
                Err(_) => last_error = Some("connection attempt timed out".to_string()),
            }

            if !announced_wait {
                log::info!(
                    "SSH connected to {}. Waiting for `marina serve` on remote 127.0.0.1:{remote_port}",
                    self.endpoint.display_host_port()
                );
                announced_wait = true;
            }
            tokio::time::sleep(REMOTE_FORWARD_RETRY_INTERVAL).await;
        }

        Err(anyhow!(
            "Remote 127.0.0.1:{remote_port} on {} stayed unreachable for {}s. Start `marina serve` there. Last error: {}",
            self.endpoint.display_host_port(),
            REMOTE_FORWARD_READY_TIMEOUT.as_secs(),
            last_error.unwrap_or_else(|| "unknown".to_string())
        ))
    }

    /// The same forward, delegated to the `ssh` binary.
    ///
    /// Used when the registry is configured for the OpenSSH transport, so that
    /// agent forwarding, `~/.ssh/config`, and everything else the user has set
    /// up keeps working with the configured tunnel.
    #[cfg(feature = "minot-registry")]
    async fn open_local_forward_openssh(&self, remote_port: u16) -> Result<SshTunnel> {
        self.ensure_openssh_auth_supported()?;
        let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
            .await
            .context("failed to reserve a local port for the ssh tunnel")?;
        let local_port = listener
            .local_addr()
            .context("failed to read the tunnel's local address")?
            .port();
        // Release the listener so SSH can take the port.
        drop(listener);

        let mut command = Command::new("ssh");
        self.add_openssh_args(&mut command, true);
        command
            .arg("-N")
            .arg("-L")
            .arg(format!("127.0.0.1:{local_port}:127.0.0.1:{remote_port}"))
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::piped());
        let mut child = command
            .spawn()
            .context("failed to start `ssh` for the tunnel")?;

        let started = Instant::now();
        let mut announced_wait = false;
        loop {
            if let Some(status) = child
                .try_wait()
                .context("failed to inspect the ssh tunnel process")?
            {
                let mut stderr = String::new();
                if let Some(mut pipe) = child.stderr.take() {
                    pipe.read_to_string(&mut stderr).await.ok();
                }
                return Err(anyhow!(
                    "ssh tunnel exited before remote 127.0.0.1:{remote_port} became reachable ({status}): {}",
                    stderr.trim()
                ));
            }

            if local_forward_destination_is_open(local_port).await {
                break;
            }
            if started.elapsed() >= REMOTE_FORWARD_READY_TIMEOUT {
                let _ = child.start_kill();
                return Err(anyhow!(
                    "Remote 127.0.0.1:{remote_port} on {} stayed unreachable for {}s. Start `marina serve` there",
                    self.endpoint.display_host_port(),
                    REMOTE_FORWARD_READY_TIMEOUT.as_secs()
                ));
            }
            if !announced_wait {
                log::info!(
                    "SSH connected to {}. Waiting for `marina serve` on remote 127.0.0.1:{remote_port}",
                    self.endpoint.display_host_port()
                );
                announced_wait = true;
            }
            tokio::time::sleep(REMOTE_FORWARD_RETRY_INTERVAL).await;
        }

        let stderr_drain = child.stderr.take().map(|mut stderr| {
            tokio::spawn(async move {
                let _ = tokio::io::copy(&mut stderr, &mut tokio::io::sink()).await;
            })
        });

        log::info!(
            "ssh tunnel (openssh): 127.0.0.1:{local_port} -> {}:{remote_port}",
            self.endpoint.display_host_port()
        );
        Ok(SshTunnel {
            local_port,
            accept: None,
            child: Some(child),
            stderr_drain,
        })
    }

    async fn get_handle(&self) -> Result<Arc<Handle<ClientHandler>>> {
        let mut guard = self.pool.lock().await;
        if let Some(h) = guard.as_ref() {
            return Ok(Arc::clone(h));
        }
        let handle = self.connect_inner().await?;
        let arc = Arc::new(handle);
        *guard = Some(Arc::clone(&arc));
        Ok(arc)
    }

    async fn get_handle_fresh(&self) -> Result<Arc<Handle<ClientHandler>>> {
        let mut guard = self.pool.lock().await;
        let handle = self.connect_inner().await?;
        let arc = Arc::new(handle);
        *guard = Some(Arc::clone(&arc));
        Ok(arc)
    }

    async fn channel_open_session(&self) -> Result<russh::Channel<client::Msg>> {
        let handle = self.get_handle().await?;
        if let Ok(ch) = handle.channel_open_session().await {
            return Ok(ch);
        }
        // Stale connection — reconnect once
        let handle = self.get_handle_fresh().await?;
        handle
            .channel_open_session()
            .await
            .context("failed opening ssh channel")
    }

    async fn run_ssh(&self, remote_cmd: &str) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            self.ensure_openssh_auth_supported()?;
            let output = self.openssh_command("ssh", remote_cmd).output().await?;
            if !output.status.success() {
                return Err(anyhow!(
                    "ssh command failed (exit {}): {}",
                    output.status.code().unwrap_or(-1),
                    String::from_utf8_lossy(&output.stderr).trim()
                ));
            }
            return Ok(());
        }

        let mut channel = self.channel_open_session().await?;
        channel
            .exec(true, remote_cmd)
            .await
            .with_context(|| format!("failed to exec remote command: {}", remote_cmd))?;

        let mut stderr = Vec::new();
        let mut exit_code: u32 = 0;

        loop {
            match channel.wait().await {
                Some(ChannelMsg::Data { .. }) => {}
                Some(ChannelMsg::ExtendedData { data, .. }) => {
                    stderr.extend_from_slice(&data);
                }
                Some(ChannelMsg::ExitStatus { exit_status }) => {
                    exit_code = exit_status;
                }
                None => break,
                _ => {}
            }
        }

        if exit_code != 0 {
            let stderr_str = String::from_utf8_lossy(&stderr).to_string();
            return Err(anyhow!(
                "ssh command failed (exit {}): {}",
                exit_code,
                stderr_str.trim()
            ));
        }
        Ok(())
    }

    async fn run_ssh_capture(&self, remote_cmd: &str) -> Result<String> {
        if self.transport == SshTransport::OpenSsh {
            self.ensure_openssh_auth_supported()?;
            let output = self.openssh_command("ssh", remote_cmd).output().await?;
            if !output.status.success() {
                return Err(anyhow!(
                    "ssh command failed (exit {}): {}",
                    output.status.code().unwrap_or(-1),
                    String::from_utf8_lossy(&output.stderr).trim()
                ));
            }
            return String::from_utf8(output.stdout).context("remote stdout was not valid UTF-8");
        }

        let mut channel = self.channel_open_session().await?;
        channel
            .exec(true, remote_cmd)
            .await
            .with_context(|| format!("failed to exec remote command: {}", remote_cmd))?;

        let mut stdout = Vec::new();
        let mut stderr = Vec::new();
        let mut exit_code: u32 = 0;

        loop {
            match channel.wait().await {
                Some(ChannelMsg::Data { data }) => {
                    stdout.extend_from_slice(&data);
                }
                Some(ChannelMsg::ExtendedData { data, .. }) => {
                    stderr.extend_from_slice(&data);
                }
                Some(ChannelMsg::ExitStatus { exit_status }) => {
                    exit_code = exit_status;
                }
                None => break,
                _ => {}
            }
        }

        if exit_code != 0 {
            let stderr_str = String::from_utf8_lossy(&stderr).to_string();
            return Err(anyhow!(
                "ssh command failed (exit {}): {}",
                exit_code,
                stderr_str.trim()
            ));
        }

        String::from_utf8(stdout).context("remote stdout was not valid UTF-8")
    }

    async fn open_sftp(&self) -> Result<SftpSession> {
        let channel = self.channel_open_session().await?;
        channel
            .request_subsystem(true, "sftp")
            .await
            .context("failed requesting sftp subsystem")?;
        SftpSession::new(channel.into_stream())
            .await
            .context("failed creating sftp session")
    }

    async fn sftp_upload(
        sftp: &SftpSession,
        local: &Path,
        remote_path: &str,
        label: Option<&str>,
    ) -> Result<()> {
        let size = fs::metadata(local)
            .with_context(|| format!("failed to stat {}", local.display()))?
            .len();
        let mut local_file = tokio::fs::File::open(local)
            .await
            .with_context(|| format!("failed opening local file {}", local.display()))?;
        let mut remote_file = sftp
            .create(remote_path)
            .await
            .with_context(|| format!("failed opening remote sftp file {}", remote_path))?;
        let pb = label.map(|l| transfer_bar(size, l));
        let mut buf = [0u8; 64 * 1024];
        let mut read = 0u64;
        loop {
            let n = local_file.read(&mut buf).await?;
            if n == 0 {
                break;
            }
            remote_file.write_all(&buf[..n]).await?;
            read += n as u64;
            if let Some(ref pb) = pb {
                pb.inc(n as u64);
            }
        }
        remote_file
            .flush()
            .await
            .with_context(|| format!("failed flushing remote sftp file {}", remote_path))?;
        remote_file
            .shutdown()
            .await
            .with_context(|| format!("failed closing remote sftp file {}", remote_path))?;
        if let Some(pb) = pb {
            pb.finish_and_clear();
        }
        if read != size {
            return Err(anyhow!(
                "incomplete local read for {}: read {} of {} bytes",
                local.display(),
                read,
                size
            ));
        }
        Ok(())
    }

    async fn sftp_download(
        sftp: &SftpSession,
        remote_path: &str,
        local: &Path,
        label: Option<&str>,
    ) -> Result<()> {
        let size = sftp
            .metadata(remote_path)
            .await
            .with_context(|| format!("failed to stat remote file {}", remote_path))?
            .size
            .unwrap_or(0);
        if let Some(parent) = local.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut local_file = tokio::fs::File::create(local)
            .await
            .with_context(|| format!("failed creating local file {}", local.display()))?;
        let mut remote_file = sftp
            .open(remote_path)
            .await
            .with_context(|| format!("failed opening remote sftp file {}", remote_path))?;
        let pb = label.map(|l| transfer_bar(size, l));
        let mut buf = [0u8; 64 * 1024];
        let mut written = 0u64;
        loop {
            let n = remote_file.read(&mut buf).await?;
            if n == 0 {
                break;
            }
            local_file.write_all(&buf[..n]).await?;
            written += n as u64;
            if let Some(ref pb) = pb {
                pb.inc(n as u64);
            }
        }
        local_file
            .sync_all()
            .await
            .with_context(|| format!("failed syncing local file {}", local.display()))?;
        if let Some(pb) = pb {
            pb.finish_and_clear();
        }
        if size != 0 && written != size {
            return Err(anyhow!(
                "incomplete sftp download for {}: received {} of {} bytes",
                remote_path,
                written,
                size
            ));
        }
        Ok(())
    }

    async fn upload_file_with_progress(&self, local: &Path, remote_path: &str) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            return self
                .openssh_copy(local, remote_path, CopyDirection::Upload)
                .await;
        }
        let sftp = self.open_sftp().await?;
        Self::sftp_upload(
            &sftp,
            local,
            remote_path,
            Some(&format!("ssh upload {}", local.display())),
        )
        .await
    }

    async fn download_file_with_progress(
        &self,
        remote_path: &str,
        local: &Path,
        label: Option<&str>,
    ) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            return self
                .openssh_copy(local, remote_path, CopyDirection::Download)
                .await;
        }
        let sftp = self.open_sftp().await?;
        Self::sftp_download(&sftp, remote_path, local, label).await
    }

    fn openssh_command(&self, program: &str, remote_cmd: &str) -> Command {
        let mut cmd = Command::new(program);
        self.add_openssh_args(&mut cmd, true);
        cmd.arg(&self.endpoint.user_host).arg(remote_cmd);
        cmd
    }

    fn add_openssh_args(&self, cmd: &mut Command, for_ssh: bool) {
        if for_ssh {
            cmd.arg("-p").arg(self.endpoint.port.to_string());
        } else {
            cmd.arg("-P").arg(self.endpoint.port.to_string());
        }
        if let Some(var) = &self.auth_env {
            if let Ok(secret) = std::env::var(var) {
                let secret_path = Path::new(&secret);
                if secret_path.exists() {
                    cmd.arg("-i").arg(secret_path);
                } else if self.transport == SshTransport::OpenSsh {
                    // The OpenSSH transport is intentionally non-interactive in Marina.
                    // Password auth would require sshpass/askpass plumbing, so keep it explicit.
                    cmd.arg("-o").arg("IdentitiesOnly=yes");
                }
            }
        }
        if let Some(proxy_jump) = &self.proxy_jump {
            cmd.arg("-J").arg(proxy_jump.open_ssh_jump_arg());
        }
        cmd.arg("-o").arg("BatchMode=yes");
    }

    async fn openssh_copy(
        &self,
        local: &Path,
        remote_path: &str,
        direction: CopyDirection,
    ) -> Result<()> {
        self.ensure_openssh_auth_supported()?;
        if let CopyDirection::Download = direction {
            if let Some(parent) = local.parent() {
                fs::create_dir_all(parent)?;
            }
        }
        let mut cmd = Command::new("scp");
        self.add_openssh_args(&mut cmd, false);
        let remote = format!("{}:{}", self.endpoint.user_host, remote_path);
        match direction {
            CopyDirection::Upload => {
                cmd.arg(local).arg(remote);
            }
            CopyDirection::Download => {
                cmd.arg(remote).arg(local);
            }
        }
        let output = cmd.output().await?;
        if !output.status.success() {
            return Err(anyhow!(
                "scp failed (exit {}): {}",
                output.status.code().unwrap_or(-1),
                String::from_utf8_lossy(&output.stderr).trim()
            ));
        }
        Ok(())
    }

    async fn remote_file_size(&self, remote_path: &str) -> Result<u64> {
        if self.transport == SshTransport::OpenSsh {
            let output = self
                .run_ssh_capture(&format!("wc -c < {}", shell_quote(remote_path)))
                .await
                .with_context(|| format!("failed to stat remote file {}", remote_path))?;
            return output.trim().parse::<u64>().with_context(|| {
                format!(
                    "remote file size for {} was not an integer: {}",
                    remote_path,
                    output.trim()
                )
            });
        }

        let sftp = self.open_sftp().await?;
        Ok(sftp
            .metadata(remote_path)
            .await
            .with_context(|| format!("failed to stat remote file {}", remote_path))?
            .size
            .unwrap_or(0))
    }

    async fn remove_remote_file(&self, remote_path: &str) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            return self
                .run_ssh(&format!("rm -f {}", shell_quote(remote_path)))
                .await;
        }

        let sftp = self.open_sftp().await?;
        let _ = sftp.remove_file(remote_path).await;
        Ok(())
    }

    async fn rename_remote_file(&self, from: &str, to: &str) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            return self
                .run_ssh(&format!("mv -f {} {}", shell_quote(from), shell_quote(to)))
                .await;
        }

        let sftp = self.open_sftp().await?;
        let _ = sftp.remove_file(to).await;
        sftp.rename(from, to)
            .await
            .with_context(|| format!("failed to rename remote file {} to {}", from, to))
    }

    async fn ensure_remote_dir(&self, remote_dir: &str) -> Result<()> {
        if self.transport == SshTransport::OpenSsh {
            return self
                .run_ssh(&format!("mkdir -p {}", shell_quote(remote_dir)))
                .await;
        }

        let sftp = self.open_sftp().await?;
        let mut current = if remote_dir.starts_with('/') {
            String::from("/")
        } else {
            String::new()
        };
        for part in remote_dir.split('/').filter(|part| !part.is_empty()) {
            if current != "/" && !current.is_empty() {
                current.push('/');
            }
            current.push_str(part);
            let _ = sftp.create_dir(current.clone()).await;
        }
        Ok(())
    }

    fn ensure_openssh_auth_supported(&self) -> Result<()> {
        if let Some(var) = &self.auth_env {
            let secret = std::env::var(var)
                .with_context(|| format!("missing ssh auth env var '{}'", var))?;
            if !Path::new(&secret).exists() {
                return Err(anyhow!(
                    "OpenSSH expects '{}' to contain a key file path. Select the native SSH transport for password authentication",
                    var
                ));
            }
        }
        Ok(())
    }

    pub(crate) async fn run_remote_capture(&self, remote_cmd: &str) -> Result<String> {
        self.run_ssh_capture(remote_cmd).await
    }

    pub(crate) async fn sync_directory_native(
        &self,
        local_root: &Path,
        remote_root: &str,
        local_files: &[MirrorFile],
        remote_files: &[MirrorFile],
    ) -> Result<()> {
        let sftp = self.open_sftp().await?;
        let remote_by_path: HashMap<&str, &MirrorFile> = remote_files
            .iter()
            .map(|file| (file.path.as_str(), file))
            .collect();

        for entry in walkdir::WalkDir::new(local_root)
            .min_depth(1)
            .follow_links(false)
        {
            let entry = entry?;
            let relative = entry.path().strip_prefix(local_root)?;
            let relative = relative.to_string_lossy().replace('\\', "/");
            let remote_path = format!("{}/{}", remote_root.trim_end_matches('/'), relative);
            if entry.file_type().is_dir() {
                self.ensure_sftp_dir(&sftp, &remote_path).await?;
            } else if entry.file_type().is_symlink() {
                return Err(anyhow!(
                    "symbolic links require rsync for cache mirroring: {}",
                    entry.path().display()
                ));
            }
        }

        for file in local_files {
            let unchanged = remote_by_path
                .get(file.path.as_str())
                .is_some_and(|remote| remote.size == file.size && remote.sha256 == file.sha256);
            if unchanged {
                continue;
            }
            let local_path = local_root.join(&file.path);
            let remote_path = format!("{}/{}", remote_root.trim_end_matches('/'), file.path);
            if let Some(parent) = remote_path.rsplit_once('/').map(|(parent, _)| parent) {
                self.ensure_sftp_dir(&sftp, parent).await?;
            }
            let temporary_path = format!("{}.mirror-uploading-{}", remote_path, std::process::id());
            let _ = sftp.remove_file(temporary_path.clone()).await;
            Self::sftp_upload(
                &sftp,
                &local_path,
                &temporary_path,
                Some(&format!("mirror {}", file.path)),
            )
            .await?;
            let _ = sftp.remove_file(remote_path.clone()).await;
            sftp.rename(temporary_path, remote_path)
                .await
                .with_context(|| format!("failed installing mirrored file {}", file.path))?;
        }

        self.upload_mirror_manifest(remote_root, local_files).await
    }

    pub(crate) async fn upload_mirror_manifest(
        &self,
        remote_root: &str,
        local_files: &[MirrorFile],
    ) -> Result<()> {
        let sftp = self.open_sftp().await?;
        let manifest_path = std::env::temp_dir().join(format!(
            "marina-mirror-manifest-{}.json",
            std::process::id()
        ));
        fs::write(&manifest_path, serde_json::to_vec(local_files)?)?;
        let remote_manifest = format!(
            "{}/.marina-mirror-manifest.json",
            remote_root.trim_end_matches('/')
        );
        let upload_result = Self::sftp_upload(&sftp, &manifest_path, &remote_manifest, None).await;
        let _ = fs::remove_file(manifest_path);
        upload_result
    }

    async fn ensure_sftp_dir(&self, sftp: &SftpSession, remote_dir: &str) -> Result<()> {
        let mut current = if remote_dir.starts_with('/') {
            String::from("/")
        } else {
            String::new()
        };
        for part in remote_dir.split('/').filter(|part| !part.is_empty()) {
            if current != "/" && !current.is_empty() {
                current.push('/');
            }
            current.push_str(part);
            if !sftp.try_exists(current.clone()).await.unwrap_or(false) {
                sftp.create_dir(current.clone())
                    .await
                    .with_context(|| format!("failed creating remote directory {}", current))?;
            }
        }
        Ok(())
    }

    pub(crate) async fn try_rsync_directory(
        &self,
        local_root: &Path,
        remote_root: &str,
    ) -> Result<bool> {
        if self.ensure_openssh_auth_supported().is_err()
            || Command::new("rsync")
                .arg("--version")
                .output()
                .await
                .map(|output| !output.status.success())
                .unwrap_or(true)
            || self
                .run_ssh("command -v rsync >/dev/null 2>&1")
                .await
                .is_err()
        {
            return Ok(false);
        }

        let mut probe = self.openssh_command("ssh", "true");
        if !probe
            .output()
            .await
            .map(|o| o.status.success())
            .unwrap_or(false)
        {
            return Ok(false);
        }

        let mut rsh = format!("ssh -p {} -o BatchMode=yes", self.endpoint.port);
        if let Some(var) = &self.auth_env {
            if let Ok(secret) = std::env::var(var) {
                if Path::new(&secret).exists() {
                    rsh.push_str(" -i ");
                    rsh.push_str(&shell_quote(&secret));
                }
            }
        }
        if let Some(proxy) = &self.proxy_jump {
            rsh.push_str(" -J ");
            rsh.push_str(&shell_quote(&proxy.open_ssh_jump_arg()));
        }

        let source = format!("{}/", local_root.display());
        let destination = format!(
            "{}:{}/",
            self.endpoint.user_host,
            shell_quote(remote_root.trim_end_matches('/'))
        );
        let mut cmd = Command::new("rsync");
        cmd.arg("-a").arg("--delete");
        if std::io::stdout().is_terminal() {
            cmd.arg("--info=progress2").arg("--human-readable");
        }
        let status = cmd
            .arg("-e")
            .arg(rsh)
            .arg(source)
            .arg(destination)
            .status()
            .await?;
        Ok(status.success())
    }

    /// Fetch all MetaFile records from the registry in a single SSH command.
    ///
    /// Uses ASCII record separator (0x1e) between files and unit separator
    /// (0x1f) between path and content. Neither can appear unescaped in valid
    /// JSON text.
    async fn fetch_all_meta(&self) -> Result<Vec<MetaFile>> {
        let cmd = format!(
            "find {} -type f -name metadata.json -exec sh -c 'for f; do printf \"\\036%s\\037\" \"$f\"; cat \"$f\"; done' _ {{}} +",
            shell_quote(&self.endpoint.root)
        );
        let output = self.run_ssh_capture(&cmd).await?;
        parse_meta_listing(&output)
    }
}

/// OpenSSH accepts a connection to its local listener before attempting the
/// far-side connection. A destination that is down closes immediately. A
/// live Zenoh listener leaves the idle connection open.
#[cfg(feature = "minot-registry")]
async fn local_forward_destination_is_open(local_port: u16) -> bool {
    let Ok(Ok(mut stream)) = tokio::time::timeout(
        Duration::from_secs(1),
        tokio::net::TcpStream::connect(("127.0.0.1", local_port)),
    )
    .await
    else {
        return false;
    };

    let mut byte = [0_u8; 1];
    matches!(
        tokio::time::timeout(Duration::from_millis(200), stream.read(&mut byte)).await,
        Err(_)
    )
}

fn parse_meta_listing(output: &str) -> Result<Vec<MetaFile>> {
    let mut metas = Vec::new();
    for record in output.split('\x1e') {
        let record = record.trim();
        if record.is_empty() {
            continue;
        }
        let (path, chunk) = record
            .split_once('\x1f')
            .ok_or_else(|| anyhow!("malformed ssh metadata listing record"))?;
        let chunk = chunk.trim();
        if chunk.is_empty() {
            continue;
        }
        let meta = serde_json::from_str::<MetaFile>(chunk)
            .with_context(|| format!("failed parsing remote metadata {}", path))?;
        metas.push(meta);
    }
    Ok(metas)
}

use async_trait::async_trait;

#[async_trait]
impl RegistryDriver for SshRegistry {
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
        self.ensure_remote_dir(&target_dir).await?;

        let tmp = std::env::temp_dir().join(format!("marina_meta_{}.json", bag.cache_key()));
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
        fs::write(&tmp, serde_json::to_vec_pretty(&meta_file)?)?;

        let data_path = self.data_path(bag);
        let meta_path = self.meta_path(bag);
        let remote_tmp_suffix = format!(".uploading.{}", std::process::id());
        let data_tmp_path = format!("{data_path}{remote_tmp_suffix}");
        let meta_tmp_path = format!("{meta_path}{remote_tmp_suffix}");

        self.remove_remote_file(&data_tmp_path).await?;
        self.remove_remote_file(&meta_tmp_path).await?;

        self.upload_file_with_progress(packed_file, &data_tmp_path)
            .await?;
        let remote_bytes = self.remote_file_size(&data_tmp_path).await?;
        if remote_bytes != meta.packed_bytes {
            let _ = self.remove_remote_file(&data_tmp_path).await;
            return Err(anyhow!(
                "incomplete upload for {}: remote has {} of {} bytes",
                bag.without_attachment(),
                remote_bytes,
                meta.packed_bytes
            ));
        }
        self.rename_remote_file(&data_tmp_path, &data_path).await?;

        self.upload_file_with_progress(&tmp, &meta_tmp_path).await?;
        self.rename_remote_file(&meta_tmp_path, &meta_path).await?;
        let _ = fs::remove_file(tmp);
        Ok(())
    }

    async fn bag_info(&self, bag: &BagRef) -> Result<Option<BagInfo>> {
        let meta_text = self
            .run_ssh_capture(&format!("cat {}", shell_quote(&self.meta_path(bag))))
            .await?;
        let meta: MetaFile = serde_json::from_str(&meta_text)?;
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
        let parent = out_packed_file
            .parent()
            .ok_or_else(|| anyhow!("invalid output path"))?;
        fs::create_dir_all(parent)?;

        let meta_local = parent.join("remote_metadata.json");
        self.download_file_with_progress(&self.meta_path(bag), &meta_local, None)
            .await?;
        let meta_text = fs::read_to_string(&meta_local)?;
        let _ = fs::remove_file(meta_local);
        let meta: MetaFile = serde_json::from_str(&meta_text)?;

        self.download_file_with_progress(
            &self.data_path(bag),
            out_packed_file,
            Some(&bag.without_attachment().to_string()),
        )
        .await?;

        let local_bytes = fs::metadata(out_packed_file)
            .with_context(|| format!("failed to stat {}", out_packed_file.display()))?
            .len();
        if local_bytes != meta.packed_bytes {
            return Err(anyhow!(
                "incomplete download for {}: received {} of {} bytes",
                bag.without_attachment(),
                local_bytes,
                meta.packed_bytes
            ));
        }

        Ok(RemoteDescriptor {
            registry_name: self.name.clone(),
            bag: meta.bag,
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
        })
    }

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;
        Ok(self
            .fetch_all_meta()
            .await?
            .into_iter()
            .map(|m| m.bag.without_attachment())
            .filter(|b: &BagRef| pattern.matches(&b.to_string()))
            .collect())
    }

    async fn list_with_info(&self, filter: &str) -> Result<Vec<(BagRef, Option<BagInfo>)>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;
        Ok(self
            .fetch_all_meta()
            .await?
            .into_iter()
            .map(|meta| {
                let bag = meta.bag.without_attachment();
                let info = BagInfo {
                    bundle_hash: meta.bundle_hash,
                    original_bytes: meta.original_bytes,
                    packed_bytes: meta.packed_bytes,
                    pointcloud: meta.pointcloud,
                    mcap_compression: meta.mcap_compression,
                    pushed_at: meta.pushed_at,
                };
                (bag, Some(info))
            })
            .filter(|(b, _): &(BagRef, Option<BagInfo>)| pattern.matches(&b.to_string()))
            .collect())
    }

    async fn remove(&self, bag: &BagRef) -> Result<()> {
        let target_dir = self.object_dir(bag);
        self.run_ssh(&format!("rm -rf {}", shell_quote(&target_dir)))
            .await
    }

    async fn write_http_index(&self) -> Result<()> {
        let output = self
            .run_ssh_capture(&format!(
                "find {} -type f -name metadata.json",
                shell_quote(&self.endpoint.root)
            ))
            .await?;

        let mut bags = Vec::new();
        for line in output.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let meta_json = self
                .run_ssh_capture(&format!("cat {}", shell_quote(line)))
                .await?;
            let meta: MetaFile = serde_json::from_str(&meta_json)
                .with_context(|| format!("failed to parse metadata at remote path {}", line))?;
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
        let tmp = std::env::temp_dir().join(format!("marina_http_index_{}.json", self.name));
        fs::write(&tmp, serde_json::to_vec_pretty(&index)?)?;
        let remote = format!("{}/index.json", self.endpoint.root);
        self.upload_file_with_progress(&tmp, &remote).await?;
        let _ = fs::remove_file(tmp);
        Ok(())
    }

    async fn check_write_access(&self) -> Result<()> {
        let probe = format!(
            "{}/.marina_write_probe_{}",
            self.endpoint.root,
            std::process::id()
        );
        self.run_ssh(&format!(
            "mkdir -p {} && rmdir {}",
            shell_quote(&probe),
            shell_quote(&probe)
        ))
        .await
    }
}

impl SshEndpoint {
    fn parse(uri: &str) -> Result<Self> {
        let raw = uri
            .strip_prefix("ssh://")
            .ok_or_else(|| anyhow!("ssh registry URI must start with ssh://"))?;

        let (authority, path) = if let Some(idx) = raw.find('/') {
            (&raw[..idx], &raw[idx..])
        } else {
            (raw, "")
        };

        if authority.is_empty() {
            return Err(anyhow!("ssh registry URI missing host"));
        }

        let (user_host, port) = parse_authority(authority)?;

        let root = if path.is_empty() {
            "~/marina-registry".to_string()
        } else {
            path.to_string()
        };

        Ok(Self {
            user_host,
            port,
            root,
        })
    }

    fn parse_jump(jump: &str) -> Result<Self> {
        let raw = jump.strip_prefix("ssh://").unwrap_or(jump);
        if raw.contains('/') {
            return Err(anyhow!(
                "ssh proxy_jump must be user@host[:port], without a path"
            ));
        }
        if raw.is_empty() {
            return Err(anyhow!("ssh proxy_jump must not be empty"));
        }
        let (user_host, port) = parse_authority(raw)?;
        Ok(Self {
            user_host,
            port,
            root: String::new(),
        })
    }

    fn display_host_port(&self) -> String {
        match split_user_host(&self.user_host) {
            Ok((_user, host)) => format!("{}:{}", host, self.port),
            Err(_) => format!("{}:{}", self.user_host, self.port),
        }
    }

    fn open_ssh_jump_arg(&self) -> String {
        if self.port == 22 {
            self.user_host.clone()
        } else {
            format!("{}:{}", self.user_host, self.port)
        }
    }
}

async fn connect_endpoint(
    config: Arc<Config>,
    endpoint: &SshEndpoint,
) -> Result<Handle<ClientHandler>> {
    let (_user, host) = split_user_host(&endpoint.user_host)?;
    client::connect(config, (host.as_str(), endpoint.port), ClientHandler).await
}

fn parse_transport(value: Option<&str>) -> Result<SshTransport> {
    match value.unwrap_or("native").to_ascii_lowercase().as_str() {
        "native" | "russh" => Ok(SshTransport::Native),
        "openssh" | "ssh" => Ok(SshTransport::OpenSsh),
        other => Err(anyhow!(
            "unsupported ssh_transport '{}'. Choose 'native' or 'openssh'",
            other
        )),
    }
}

fn parse_authority(authority: &str) -> Result<(String, u16)> {
    if let Some((left, right)) = authority.rsplit_once(':') {
        if !left.is_empty() && !right.is_empty() && right.chars().all(|c| c.is_ascii_digit()) {
            let port: u16 = right
                .parse()
                .with_context(|| format!("invalid ssh port '{}'", right))?;
            return Ok((left.to_string(), port));
        }
        // Trailing colon with no port (e.g. "host:/path") — strip the colon
        if right.is_empty() {
            return Ok((left.to_string(), 22));
        }
    }
    Ok((authority.to_string(), 22))
}

fn split_user_host(user_host: &str) -> Result<(String, String)> {
    if let Some((user, host)) = user_host.split_once('@') {
        if user.is_empty() || host.is_empty() {
            return Err(anyhow!("invalid ssh authority '{}'", user_host));
        }
        return Ok((user.to_string(), host.to_string()));
    }

    let user = std::env::var("USER").context("missing USER env var for ssh auth")?;
    Ok((user, user_host.to_string()))
}

fn shell_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', "'\\''"))
}

fn transfer_bar(total: u64, message: &str) -> ProgressBar {
    let pb = if total > 0 {
        ProgressBar::new(total)
    } else {
        ProgressBar::new_spinner()
    };
    if !std::io::stdout().is_terminal() {
        pb.set_draw_target(ProgressDrawTarget::hidden());
    }
    pb.set_style(
        ProgressStyle::with_template(
            "{msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes} {bytes_per_sec} eta {eta}",
        )
        .unwrap_or_else(|_| ProgressStyle::default_bar()),
    );
    pb.enable_steady_tick(std::time::Duration::from_millis(100));
    pb.set_message(message.to_string());
    pb
}
