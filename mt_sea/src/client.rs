use std::collections::HashMap;

use anyhow::anyhow;
use log::{debug, error, info, warn};
use mt_net::COMPARE_NODE_NAME;
use rkyv::{api::high::from_bytes, to_bytes, util::AlignedVec};
use zenoh::Wait;

#[cfg(feature = "shm")]
use std::sync::RwLock as StdRwLock;
#[cfg(feature = "shm")]
use zenoh::shm::{
    BlockOn, GarbageCollect, PosixShmProviderBackend, ShmProvider, ShmProviderBuilder,
};

use crate::{
    ShipKind, ShipName, VariableType,
    net::{CONTROLLER_CLIENT_ID, Packet, PacketKind, Qos, Sea, get_domain_id, sanitize_key},
};

pub type CoordSender = tokio::sync::broadcast::Sender<(Packet, Option<std::net::SocketAddr>)>;
pub type RecvBuffer = HashMap<u32, Vec<(AlignedVec, VariableType, String)>>;

#[cfg(feature = "shm")]
/// Large enough for common image payloads while remaining conservative for 64 MiB containers.
const DEFAULT_SHM_BUFFER_SIZE: usize = 16 * 1024 * 1024;

#[cfg(feature = "shm")]
const DEFAULT_SHM_SIZE_THRESHOLD: usize = 1024 * 1024;

#[cfg(feature = "shm")]
const DEFAULT_SHM_MAX_MESSAGE_SIZE: usize = 64 * 1024 * 1024;

#[cfg(feature = "shm")]
const DEFAULT_SHM_ALLOCATION_TIMEOUT_MS: u64 = 250;

#[cfg(feature = "shm")]
static SHM_SEND_ATTEMPTS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static SHM_SEND_SUCCESSES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static SHM_SEND_FALLBACKS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static SHM_RECEIVES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static SHM_SEND_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static SHM_RECEIVE_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static NETWORK_SENDS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static NETWORK_SEND_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static NETWORK_RECEIVES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
#[cfg(feature = "shm")]
static NETWORK_RECEIVE_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Process-local counters for observing whether large payloads actually use shared memory.
#[cfg(feature = "shm")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ShmTransferStats {
    pub send_attempts: u64,
    pub send_successes: u64,
    pub send_fallbacks: u64,
    pub receives: u64,
    pub send_bytes: u64,
    pub receive_bytes: u64,
    pub network_sends: u64,
    pub network_send_bytes: u64,
    pub network_receives: u64,
    pub network_receive_bytes: u64,
}

#[cfg(feature = "shm")]
impl ShmTransferStats {
    /// Calculate the traffic observed since an earlier snapshot.
    pub fn since(self, earlier: Self) -> Self {
        Self {
            send_attempts: self.send_attempts.saturating_sub(earlier.send_attempts),
            send_successes: self.send_successes.saturating_sub(earlier.send_successes),
            send_fallbacks: self.send_fallbacks.saturating_sub(earlier.send_fallbacks),
            receives: self.receives.saturating_sub(earlier.receives),
            send_bytes: self.send_bytes.saturating_sub(earlier.send_bytes),
            receive_bytes: self.receive_bytes.saturating_sub(earlier.receive_bytes),
            network_sends: self.network_sends.saturating_sub(earlier.network_sends),
            network_send_bytes: self
                .network_send_bytes
                .saturating_sub(earlier.network_send_bytes),
            network_receives: self
                .network_receives
                .saturating_sub(earlier.network_receives),
            network_receive_bytes: self
                .network_receive_bytes
                .saturating_sub(earlier.network_receive_bytes),
        }
    }

    /// Whether the snapshot contains completed payload traffic.
    pub fn has_traffic(self) -> bool {
        self.send_successes + self.network_sends + self.receives + self.network_receives > 0
    }
}

/// Return a snapshot of the process-local shared-memory transfer counters.
#[cfg(feature = "shm")]
pub fn shm_transfer_stats() -> ShmTransferStats {
    use std::sync::atomic::Ordering::Relaxed;

    ShmTransferStats {
        send_attempts: SHM_SEND_ATTEMPTS.load(Relaxed),
        send_successes: SHM_SEND_SUCCESSES.load(Relaxed),
        send_fallbacks: SHM_SEND_FALLBACKS.load(Relaxed),
        receives: SHM_RECEIVES.load(Relaxed),
        send_bytes: SHM_SEND_BYTES.load(Relaxed),
        receive_bytes: SHM_RECEIVE_BYTES.load(Relaxed),
        network_sends: NETWORK_SENDS.load(Relaxed),
        network_send_bytes: NETWORK_SEND_BYTES.load(Relaxed),
        network_receives: NETWORK_RECEIVES.load(Relaxed),
        network_receive_bytes: NETWORK_RECEIVE_BYTES.load(Relaxed),
    }
}

/// Reset the process-local shared-memory transfer counters.
#[cfg(feature = "shm")]
pub fn reset_shm_transfer_stats() {
    use std::sync::atomic::Ordering::Relaxed;

    SHM_SEND_ATTEMPTS.store(0, Relaxed);
    SHM_SEND_SUCCESSES.store(0, Relaxed);
    SHM_SEND_FALLBACKS.store(0, Relaxed);
    SHM_RECEIVES.store(0, Relaxed);
    SHM_SEND_BYTES.store(0, Relaxed);
    SHM_RECEIVE_BYTES.store(0, Relaxed);
    NETWORK_SENDS.store(0, Relaxed);
    NETWORK_SEND_BYTES.store(0, Relaxed);
    NETWORK_RECEIVES.store(0, Relaxed);
    NETWORK_RECEIVE_BYTES.store(0, Relaxed);
}

#[cfg(feature = "shm")]
fn is_shm_enabled() -> bool {
    crate::network::is_shm_runtime_available() && !crate::network::is_shm_disabled()
}

/// SHM buffer size from the environment, or the default.
#[cfg(feature = "shm")]
fn get_shm_buffer_size() -> usize {
    std::env::var("MINOT_SHM_SIZE")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(DEFAULT_SHM_BUFFER_SIZE)
}

#[cfg(feature = "shm")]
fn get_shm_size_threshold() -> usize {
    std::env::var("MINOT_SHM_THRESHOLD")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(DEFAULT_SHM_SIZE_THRESHOLD)
}

#[cfg(feature = "shm")]
fn get_shm_max_message_size() -> usize {
    std::env::var("MINOT_SHM_MAX_MESSAGE_SIZE")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(DEFAULT_SHM_MAX_MESSAGE_SIZE)
}

#[cfg(feature = "shm")]
fn get_shm_allocation_timeout() -> std::time::Duration {
    let milliseconds = std::env::var("MINOT_SHM_ALLOCATION_TIMEOUT_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .unwrap_or(DEFAULT_SHM_ALLOCATION_TIMEOUT_MS);
    std::time::Duration::from_millis(milliseconds.max(1))
}

/// Copy bytes into an aligned buffer for rkyv deserialization.
fn align_bytes(bytes: &[u8]) -> AlignedVec {
    let mut aligned = AlignedVec::with_capacity(bytes.len());
    aligned.extend_from_slice(bytes);
    aligned
}

/// Decode one data payload into the receive buffer and wake any consumer.
///
/// Shared by the queryable, which answers the reliable modes' queries, and the
/// subscriber, which takes best-effort pushes, so both land in the same buffer
/// and `catch` cannot tell them apart.
///
/// Returns the error text to reply with when the payload is unusable. A pushed
/// message has nobody to reply to, so the caller decides what to do with it.
fn accept_data_payload(
    payload_bytes: &[u8],
    raw_recv_buff: &std::sync::Arc<std::sync::RwLock<RecvBuffer>>,
    updated_raw_recv: &tokio::sync::broadcast::Sender<u32>,
) -> Result<u32, &'static str> {
    // id (4 bytes) + variable_type (1 byte) + name (64 bytes) + data
    if payload_bytes.len() < 69 {
        return Err("payload too short");
    }

    let msg_id = u32::from_be_bytes([
        payload_bytes[0],
        payload_bytes[1],
        payload_bytes[2],
        payload_bytes[3],
    ]);
    let variable_type = VariableType::from(payload_bytes[4]);

    let name_bytes = &payload_bytes[5..69];
    let var_name =
        String::from_utf8_lossy(name_bytes.split(|&b| b == 0).next().unwrap_or_default())
            .to_string();

    // Copy only the archived data into its final, rkyv-aligned receive buffer.
    let data = align_bytes(&payload_bytes[69..]);

    {
        let mut lock = raw_recv_buff.write().unwrap();
        lock.entry(msg_id)
            .or_default()
            .push((data, variable_type, var_name));
    }

    if updated_raw_recv.send(msg_id).is_err() {
        debug!("Data for id {} ready, but no consumers listening", msg_id);
    }

    Ok(msg_id)
}

/// SHM state, resizable at runtime.
#[cfg(feature = "shm")]
struct ShmState {
    provider: std::sync::Arc<ShmProvider<PosixShmProviderBackend>>,
    capacity: usize,
}

pub struct Client {
    pub coordinator_receive: std::sync::Arc<std::sync::RwLock<Option<CoordSender>>>,
    pub coordinator_send:
        std::sync::Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<Packet>>>>,
    /// Dedicated control-plane heartbeat path. This must not share the packet
    /// queue with wind/data traffic or data-plane backpressure can look like a
    /// dead coordinator.
    coordinator_heartbeat_send:
        std::sync::Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<()>>>>,
    pub kind: ShipKind,
    rm_rules_on_disconnect: bool,
    node_mode: Qos,
    timing: crate::Timing,
    connection: std::sync::Arc<crate::ConnectionState>,
    pub updated_raw_recv: tokio::sync::broadcast::Sender<u32>,
    pub raw_recv_buff: std::sync::Arc<std::sync::RwLock<RecvBuffer>>,
    pub wind_receiver: std::sync::Arc<tokio::sync::Mutex<tokio::sync::mpsc::Receiver<Packet>>>,
    wind_sender: tokio::sync::mpsc::Sender<Packet>,
    session: std::sync::Arc<zenoh::Session>,
    domain_id: u16,
    #[cfg(feature = "shm")]
    shm_state: StdRwLock<Option<ShmState>>,
    #[cfg(feature = "shm")]
    shm_initialized: std::sync::atomic::AtomicBool,
}

impl std::fmt::Debug for Client {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Client")
            .field("kind", &self.kind)
            .field("domain_id", &self.domain_id)
            .field("rm_rules_on_disconnect", &self.rm_rules_on_disconnect)
            .field("timing", &self.timing)
            .finish_non_exhaustive()
    }
}

impl Client {
    /// The heartbeat channel itself, so a caller can reach it without taking
    /// the lock around this `Client`.
    ///
    /// The channel is filled in by `register`, and the slot is shared, so a
    /// holder taken before registration sees the sender once it exists.
    pub(crate) fn heartbeat_channel(
        &self,
    ) -> std::sync::Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<()>>>> {
        std::sync::Arc::clone(&self.coordinator_heartbeat_send)
    }
}

#[cfg(feature = "shm")]
fn create_shm_provider(size: usize) -> Result<ShmState, String> {
    match ShmProviderBuilder::default_backend(size).wait() {
        Ok(provider) => Ok(ShmState {
            provider: std::sync::Arc::new(provider),
            capacity: size,
        }),
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(feature = "shm")]
fn format_shm_error(err_str: &str, requested_size: usize) -> String {
    #[cfg(target_os = "linux")]
    {
        let shm_mb = requested_size / (1024 * 1024);
        if err_str.contains("ENOMEM")
            || err_str.contains("Cannot allocate")
            || err_str.contains("No space")
        {
            format!(
                "Insufficient shared memory. Requested {} bytes.\n\
                Possible fixes:\n\
                - Increase /dev/shm size: `sudo mount -o remount,size={}M /dev/shm`\n\
                - Or set ulimit: `ulimit -l unlimited` (may require root)\n\
                - Or add to /etc/fstab: `tmpfs /dev/shm tmpfs defaults,size={}M 0 0`\n\
                - Or disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`",
                requested_size,
                shm_mb.max(16),
                shm_mb.max(16)
            )
        } else if err_str.contains("EACCES") || err_str.contains("Permission denied") {
            "Permission denied.\n\
            Possible fixes:\n\
            - Check /dev/shm permissions: `ls -la /dev/shm`\n\
            - Ensure user has write access to /dev/shm\n\
            - Or disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`"
                .to_string()
        } else {
            format!(
                "Error: {}\nTo disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`",
                err_str
            )
        }
    }

    #[cfg(target_os = "macos")]
    {
        if err_str.contains("ENOMEM")
            || err_str.contains("Cannot allocate")
            || err_str.contains("No space")
        {
            format!(
                "Insufficient shared memory. Requested {} bytes.\n\
                Possible fixes:\n\
                - Increase shmmax: `sudo sysctl -w kern.sysv.shmmax={}`\n\
                - Increase shmall: `sudo sysctl -w kern.sysv.shmall={}`\n\
                - Or disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`\n\
                Note: macOS has stricter SHM limits than Linux.",
                requested_size,
                requested_size,
                requested_size / 4096
            )
        } else if err_str.contains("EACCES") || err_str.contains("Permission denied") {
            "Permission denied.\n\
            Possible fixes:\n\
            - Check System Preferences > Security & Privacy settings\n\
            - Or disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`"
                .to_string()
        } else {
            format!(
                "Error: {}\nTo disable SHM: `--no-shm` or `MINOT_SHM_DISABLED=1`",
                err_str
            )
        }
    }
}

impl Client {
    /// Get the SHM provider, creating it on the first call.
    #[cfg(feature = "shm")]
    fn get_or_init_shm(&self) -> Option<std::sync::Arc<ShmProvider<PosixShmProviderBackend>>> {
        if !is_shm_enabled() {
            return None;
        }

        if self
            .shm_initialized
            .load(std::sync::atomic::Ordering::Acquire)
        {
            return self
                .shm_state
                .read()
                .ok()?
                .as_ref()
                .map(|s| s.provider.clone());
        }

        let mut state = self.shm_state.write().ok()?;
        if state.is_none() {
            let initial_size = get_shm_buffer_size();
            match create_shm_provider(initial_size) {
                Ok(shm_state) => {
                    info!(
                        "SHM enabled with initial buffer size: {} bytes",
                        initial_size
                    );
                    *state = Some(shm_state);
                    self.shm_initialized
                        .store(true, std::sync::atomic::Ordering::Release);
                }
                Err(e) => {
                    warn!(
                        "Failed to create SHM provider: {}\nFalling back to network transport.",
                        format_shm_error(&e, initial_size)
                    );
                    crate::network::disable_shm_runtime();
                    return None;
                }
            }
        }
        state.as_ref().map(|s| s.provider.clone())
    }

    /// Try to grow the SHM pool to fit a larger message.
    #[cfg(feature = "shm")]
    fn try_grow_shm(
        &self,
        required_size: usize,
    ) -> Option<std::sync::Arc<ShmProvider<PosixShmProviderBackend>>> {
        let mut state = self.shm_state.write().ok()?;

        let current_capacity = state.as_ref().map(|s| s.capacity).unwrap_or(0);

        let new_capacity = required_size
            .max(current_capacity.saturating_mul(2))
            .min(get_shm_max_message_size());

        debug!(
            "Growing SHM pool from {} to {} bytes",
            current_capacity, new_capacity
        );

        match create_shm_provider(new_capacity) {
            Ok(new_state) => {
                info!("SHM pool grown to {} bytes", new_capacity);
                *state = Some(new_state);
                state.as_ref().map(|s| s.provider.clone())
            }
            Err(e) => {
                warn!(
                    "Cannot grow SHM pool to {} bytes: {}\nFalling back to network transport for large messages.",
                    new_capacity,
                    format_shm_error(&e, new_capacity)
                );
                None
            }
        }
    }

    /// Current SHM capacity.
    #[cfg(feature = "shm")]
    fn shm_capacity(&self) -> usize {
        self.shm_state
            .read()
            .ok()
            .and_then(|s| s.as_ref().map(|state| state.capacity))
            .unwrap_or(0)
    }

    /// Try to send via SHM with automatic pool growth. Returns:
    /// - Some(Ok(())) if sent successfully via SHM
    /// - Some(Err(e)) if there was an error during sending
    /// - None if SHM is unavailable and caller should fall back to network
    #[cfg(feature = "shm")]
    #[allow(clippy::too_many_arguments)]
    async fn try_shm_send(
        &self,
        total_len: usize,
        id_bytes: &[u8; 4],
        variable_type: VariableType,
        padded_name: &[u8; 64],
        data: &[u8],
        data_key: &str,
        id: u32,
    ) -> Option<anyhow::Result<()>> {
        let mut shm_provider = self.get_or_init_shm()?;

        let max_message_size = get_shm_max_message_size();
        if total_len > max_message_size {
            warn!(
                "Message size {} exceeds the {} byte SHM limit. Using network transport",
                total_len, max_message_size
            );
            return None;
        }

        let current_capacity = self.shm_capacity();
        if total_len > current_capacity {
            debug!(
                "Message ({} bytes) exceeds current SHM capacity ({} bytes), growing pool before allocation",
                total_len, current_capacity
            );
            shm_provider = match self.try_grow_shm(total_len) {
                Some(provider) => provider,
                None => {
                    warn!(
                        "Could not grow the SHM pool for {} bytes. Using network transport",
                        total_len
                    );
                    return None;
                }
            };
        }

        // Two attempts: the second runs after the pool has grown.
        for attempt in 0..2 {
            let shm_result = match tokio::time::timeout(
                get_shm_allocation_timeout(),
                shm_provider
                    .alloc(total_len)
                    .with_policy::<BlockOn<GarbageCollect>>(),
            )
            .await
            {
                Ok(result) => result,
                Err(_) => {
                    warn!(
                        "SHM allocation for {} bytes timed out. Using network transport",
                        total_len
                    );
                    return None;
                }
            };

            match shm_result {
                Ok(mut shm_buf) => {
                    shm_buf[0..4].copy_from_slice(id_bytes);
                    shm_buf[4] = variable_type.into();
                    shm_buf[5..69].copy_from_slice(padded_name);
                    shm_buf[69..total_len].copy_from_slice(data);

                    let shm_immut: zenoh::shm::ZShm = shm_buf.into();
                    debug!("Sending {} bytes via SHM to {}", total_len, data_key);

                    loop {
                        let replies =
                            match self.session.get(data_key).payload(shm_immut.clone()).wait() {
                                Ok(r) => r,
                                Err(e) => {
                                    return Some(Err(anyhow!("Failed to send data query: {}", e)));
                                }
                            };

                        match replies.recv_async().await {
                            Ok(reply) => match reply.result() {
                                Ok(_sample) => {
                                    SHM_SEND_SUCCESSES
                                        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                    SHM_SEND_BYTES.fetch_add(
                                        total_len as u64,
                                        std::sync::atomic::Ordering::Relaxed,
                                    );
                                    debug!(
                                        "Sent data id {} to {} via SHM (ACK received)",
                                        id, data_key
                                    );
                                    return Some(Ok(()));
                                }
                                Err(err) => {
                                    let err_payload = err.payload().to_bytes();
                                    let err_msg = String::from_utf8_lossy(&err_payload);
                                    warn!("Receiver error for id {}: {}", id, err_msg);
                                    tokio::task::yield_now().await;
                                    continue;
                                }
                            },
                            Err(_) => {
                                tokio::task::yield_now().await;
                                continue;
                            }
                        }
                    }
                }
                Err(_) if attempt == 0 => {
                    let current_capacity = self.shm_capacity();
                    if total_len > current_capacity {
                        debug!(
                            "Message ({} bytes) exceeds current SHM capacity ({} bytes), growing pool",
                            total_len, current_capacity
                        );
                        if let Some(new_provider) = self.try_grow_shm(total_len) {
                            shm_provider = new_provider;
                            continue;
                        }
                    }
                    warn!(
                        "SHM allocation failed for {} bytes, falling back to network transport",
                        total_len
                    );
                    return None;
                }
                Err(_) => {
                    warn!(
                        "SHM allocation failed after pool growth, falling back to network transport"
                    );
                    return None;
                }
            }
        }
        None
    }

    pub async fn init(
        ship_kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        timing: crate::Timing,
        connection: std::sync::Arc<crate::ConnectionState>,
    ) -> anyhow::Result<Self> {
        timing.validate()?;
        let domain_id = get_domain_id();
        if domain_id > 0 {
            info!("Client using domain ID {}", domain_id);
        }

        let session = crate::network::open_zenoh_session(crate::network::NetworkRole::Client)?;
        let session = std::sync::Arc::new(session);

        let (updated_raw_recv, _) = tokio::sync::broadcast::channel(100);
        let raw_recv_buff: std::sync::Arc<std::sync::RwLock<RecvBuffer>> =
            std::sync::Arc::new(std::sync::RwLock::new(HashMap::new()));

        // MPSC so wind packets are buffered rather than raced over.
        let (wind_sender, wind_receiver) = tokio::sync::mpsc::channel::<Packet>(100);
        let wind_receiver = std::sync::Arc::new(tokio::sync::Mutex::new(wind_receiver));

        let coord_send_tx = std::sync::Arc::new(std::sync::RwLock::new(None));
        let coord_heartbeat_tx = std::sync::Arc::new(std::sync::RwLock::new(None));
        let coord_receive_tx: std::sync::Arc<std::sync::RwLock<Option<CoordSender>>> =
            std::sync::Arc::new(std::sync::RwLock::new(None));

        let ship_name_str = match &ship_kind {
            ShipKind::Rat(name) => name.clone(),
            ShipKind::Wind(name) => name.clone(),
        };
        let ship_name_key = sanitize_key(&ship_name_str);

        let data_key = format!("minot/{}/data/{}", domain_id, ship_name_key);
        let updated_raw_recv_clone = updated_raw_recv.clone();
        let raw_recv_buff_clone = std::sync::Arc::clone(&raw_recv_buff);

        let queryable = session
            .declare_queryable(&data_key)
            .wait()
            .expect("Failed to create data queryable");

        debug!(
            "Client {} queryable declared on {}",
            ship_name_str, data_key
        );

        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel::<()>();

        tokio::spawn(async move {
            let _ = ready_tx.send(());

            loop {
                match queryable.recv_async().await {
                    Ok(query) => {
                        let payload_bytes: std::borrow::Cow<'_, [u8]> = match query.payload() {
                            Some(p) => {
                                #[cfg(feature = "shm")]
                                {
                                    if let Some(shm_buf) = p.as_shm() {
                                        SHM_RECEIVES
                                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                        SHM_RECEIVE_BYTES.fetch_add(
                                            shm_buf.len() as u64,
                                            std::sync::atomic::Ordering::Relaxed,
                                        );
                                        debug!("Received SHM payload");
                                        std::borrow::Cow::Borrowed(&shm_buf[..])
                                    } else {
                                        NETWORK_RECEIVES
                                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                        NETWORK_RECEIVE_BYTES.fetch_add(
                                            p.len() as u64,
                                            std::sync::atomic::Ordering::Relaxed,
                                        );
                                        p.to_bytes()
                                    }
                                }
                                #[cfg(not(feature = "shm"))]
                                {
                                    p.to_bytes()
                                }
                            }
                            None => {
                                error!("Query has no payload");
                                if let Err(e) = query.reply_err("no payload").wait() {
                                    error!("Failed to send error reply: {}", e);
                                }
                                continue;
                            }
                        };

                        if let Err(reason) = accept_data_payload(
                            &payload_bytes,
                            &raw_recv_buff_clone,
                            &updated_raw_recv_clone,
                        ) {
                            error!("Data payload rejected: {reason}");
                            if let Err(e) = query.reply_err(reason).wait() {
                                error!("Failed to send error reply: {}", e);
                            }
                            continue;
                        }

                        let key_expr = query.key_expr().clone();
                        if let Err(e) = query.reply(key_expr, &[0u8; 1]).wait() {
                            error!("Failed to send ACK reply: {}", e);
                        }
                    }
                    Err(e) => {
                        error!("Error receiving data query: {}", e);
                        break;
                    }
                }
            }
        });

        let _ = ready_rx.await;
        debug!("Client {} queryable handler ready", ship_name_str);

        // Best-effort arrives as a `put`, so it needs a subscriber beside the
        // queryable. Both sit on the same key and Zenoh routes a `put` to
        // subscribers and a `get` to queryables, so they never see each other's
        // traffic. Both feed the same buffer, so `catch` cannot tell which way
        // a sample arrived.
        //
        // The push costs the sender nothing: no reply channel to wait on,
        // abandon, or be told is closed when a peer reappears and answers a
        // query that was given up on long ago.
        let push_recv_buff = std::sync::Arc::clone(&raw_recv_buff);
        let push_updated = updated_raw_recv.clone();
        let push_subscriber = session
            .declare_subscriber(&data_key)
            .wait()
            .expect("Failed to create data subscriber");
        debug!(
            "Client {} best-effort subscriber declared on {}",
            ship_name_str, data_key
        );
        tokio::spawn(async move {
            while let Ok(sample) = push_subscriber.recv_async().await {
                let payload_bytes = sample.payload().to_bytes();
                // Counted here as well as on the queryable, or best-effort
                // traffic would be invisible in the receive metrics. Always
                // network counters, since only `put` reaches this path.
                #[cfg(feature = "shm")]
                {
                    NETWORK_RECEIVES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    NETWORK_RECEIVE_BYTES.fetch_add(
                        payload_bytes.len() as u64,
                        std::sync::atomic::Ordering::Relaxed,
                    );
                }
                if let Err(reason) =
                    accept_data_payload(&payload_bytes, &push_recv_buff, &push_updated)
                {
                    // Nobody is waiting on an answer, and one dropped
                    // best-effort sample is not an error.
                    debug!("Best-effort payload discarded: {reason}");
                }
            }
        });

        // Lets peers ping this node directly. Any query gets an empty payload
        // back.
        let heartbeat_key = format!("minot/{}/heartbeat/{}", domain_id, ship_name_key);
        let heartbeat_queryable = session
            .declare_queryable(&heartbeat_key)
            .wait()
            .expect("Failed to create heartbeat queryable");
        debug!(
            "Client {} heartbeat queryable on {}",
            ship_name_str, heartbeat_key
        );
        tokio::spawn(async move {
            while let Ok(query) = heartbeat_queryable.recv_async().await {
                let key_expr = query.key_expr().clone();
                let _ = query.reply(key_expr, &[0u8; 1]).wait();
            }
        });

        Ok(Self {
            kind: ship_kind,
            coordinator_send: coord_send_tx,
            coordinator_heartbeat_send: coord_heartbeat_tx,
            coordinator_receive: coord_receive_tx,
            rm_rules_on_disconnect,
            node_mode,
            timing,
            connection,
            updated_raw_recv,
            raw_recv_buff,
            wind_receiver,
            wind_sender,
            session,
            domain_id,
            #[cfg(feature = "shm")]
            shm_state: StdRwLock::new(None),
            #[cfg(feature = "shm")]
            shm_initialized: std::sync::atomic::AtomicBool::new(false),
        })
    }

    /// Register the client to the network.
    pub async fn register(&mut self) -> anyhow::Result<tokio::sync::oneshot::Receiver<()>> {
        let ship_name = match &self.kind {
            ShipKind::Rat(name) => name.clone(),
            ShipKind::Wind(name) => name.clone(),
        };
        let ship_name_key = sanitize_key(&ship_name);
        // A fresh registration has not yet been answered by anyone.
        self.connection.mark_link_unproven();

        let coord_to_client_key =
            format!("minot/{}/coord/clients/{}", self.domain_id, ship_name_key);
        let client_to_coord_key =
            format!("minot/{}/clients/{}/coord", self.domain_id, ship_name_key);
        let join_key = format!("minot/{}/coord/join", self.domain_id);
        let heartbeat_to_coord_key =
            crate::net::client_heartbeat_key(self.domain_id, &ship_name_key);

        let coord_subscriber = self
            .session
            .declare_subscriber(&coord_to_client_key)
            .wait()
            .expect("Failed to create coordinator subscriber");

        // Heartbeats have their own high-priority publisher/subscriber pair.
        // They bypass coordinator_send and the ordinary packet
        // publisher so a full wind/data queue cannot delay liveness.
        let heartbeat_publisher = self
            .session
            .declare_publisher(heartbeat_to_coord_key)
            .priority(zenoh::qos::Priority::RealTime)
            .congestion_control(zenoh::qos::CongestionControl::Drop)
            .reliability(zenoh::qos::Reliability::Reliable)
            .wait()
            .expect("Failed to create coordinator heartbeat publisher");
        let (heartbeat_tx, mut heartbeat_rx) = tokio::sync::mpsc::channel::<()>(1);
        self.coordinator_heartbeat_send
            .write()
            .unwrap()
            .replace(heartbeat_tx);
        tokio::spawn(async move {
            while heartbeat_rx.recv().await.is_some() {
                if let Err(error) = heartbeat_publisher.put(&[0u8; 1]).wait() {
                    debug!("Failed to send dedicated coordinator heartbeat: {error}");
                }
            }
        });

        debug!("Client {} listening on {}", ship_name, coord_to_client_key);

        let (send_tx, mut send_rx) = tokio::sync::mpsc::channel::<Packet>(256);
        let (recv_tx, _) =
            tokio::sync::broadcast::channel::<(Packet, Option<std::net::SocketAddr>)>(256);

        {
            self.coordinator_send.write().unwrap().replace(send_tx);
            self.coordinator_receive
                .write()
                .unwrap()
                .replace(recv_tx.clone());
        }

        let session_for_send = std::sync::Arc::clone(&self.session);
        let client_to_coord_key_owned = client_to_coord_key.clone();
        let node_mode = self.node_mode;
        tokio::spawn(async move {
            let coord_publisher = session_for_send
                .declare_publisher(client_to_coord_key_owned)
                .congestion_control(node_mode.congestion_control())
                .reliability(node_mode.reliability())
                .wait()
                .expect("Failed to create coordinator publisher");

            while let Some(packet) = send_rx.recv().await {
                let bytes =
                    to_bytes::<rkyv::rancor::Error>(&packet).expect("Failed to serialize packet");
                if let Err(e) = coord_publisher.put(&*bytes).wait() {
                    error!("Failed to send to coordinator: {}", e);
                    break;
                }
            }
        });

        let recv_tx_clone = recv_tx.clone();
        let ship_kind_clone = self.kind.clone();
        let wind_sender_clone = self.wind_sender.clone();
        let disconnect_timeout = self.timing.disconnect_timeout();
        let connection_for_watch = std::sync::Arc::clone(&self.connection);
        let registration_timeout = self.timing.registration_timeout();
        let (disconnect_tx, disconnect_rx) = tokio::sync::oneshot::channel::<()>();
        let (reg_done_tx, mut reg_done_rx) = tokio::sync::oneshot::channel::<()>();

        tokio::spawn(async move {
            // Phase 1: registration and wait_for_ack take as long as they take.
            loop {
                tokio::select! {
                    result = coord_subscriber.recv_async() => {
                        match result {
                            Ok(sample) => {
                                let payload = sample.payload().to_bytes();
                                let aligned = align_bytes(&payload);
                                match from_bytes::<Packet, rkyv::rancor::Error>(&aligned) {
                                    Ok(packet) => {
                                        if matches!(packet.data, PacketKind::Wind(_)) {
                                            if let Err(e) = wind_sender_clone.send(packet.clone()).await {
                                                debug!("Failed to forward wind packet: {}", e);
                                            }
                                        }
                                        if let Err(e) = recv_tx_clone.send((packet, None)) {
                                            debug!("Failed to forward coordinator packet: {}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Failed to deserialize coordinator packet: {}", e);
                                    }
                                }
                            }
                            Err(e) => {
                                warn!("Coordinator connection lost for {:?}: {}", ship_kind_clone, e);
                                let _ = disconnect_tx.send(());
                                return;
                            }
                        }
                    }
                    // Fires once register() has completed, wait_for_ack included.
                    _ = &mut reg_done_rx => break,
                }
            }

            // Phase 2: the coordinator must echo this node's heartbeats within
            // its configured disconnect timeout.
            //
            // The detector arms on the first echo, which cannot arrive before
            // this node has sent its first heartbeat, up to one interval later.
            // Until then a heartbeating node still gives up after a grace
            // period, so a coordinator that dies right after welcoming it is
            // noticed. A node that does not heartbeat expects no reply and is
            // never timed out.
            let mut received_first = false;
            // Time without any word from the coordinator while unarmed.
            let mut silent_for = tokio::time::Duration::ZERO;
            // Short before the first echo, so a node that starts heartbeating
            // just after registering is still noticed quickly.
            let unarmed_poll = tokio::time::Duration::from_millis(200).min(disconnect_timeout);
            // Also covers the coordinator still setting itself up, hence the
            // generous window.
            let startup_grace = disconnect_timeout + registration_timeout;
            loop {
                let wait = if received_first {
                    disconnect_timeout
                } else {
                    unarmed_poll
                };
                // A heartbeat already waiting in the subscriber wins over the
                // deadline, which matters when a stalled runtime makes both
                // ready at once. `timeout()` checks the timer first and caused
                // false disconnects there.
                let result = tokio::select! {
                    biased;
                    sample = coord_subscriber.recv_async() => Ok(sample),
                    _ = tokio::time::sleep(wait) => Err(()),
                };
                if result.is_ok() {
                    silent_for = tokio::time::Duration::ZERO;
                } else if !received_first {
                    // Unarmed: give up only once this node is asking for
                    // replies and has waited out the grace period.
                    silent_for += wait;
                    if !connection_for_watch.is_heartbeating() || silent_for < startup_grace {
                        continue;
                    }
                    warn!(
                        "Coordinator never answered {:?} within {:?} — treating as unreachable",
                        ship_kind_clone, startup_grace
                    );
                    let _ = disconnect_tx.send(());
                    break;
                }
                match result {
                    Ok(Ok(sample)) => {
                        let payload = sample.payload().to_bytes();
                        let aligned = align_bytes(&payload);
                        match from_bytes::<Packet, rkyv::rancor::Error>(&aligned) {
                            Ok(packet) => {
                                if matches!(packet.data, PacketKind::Heartbeat) {
                                    if !received_first {
                                        debug!(
                                            "{ship_kind_clone:?} coordinator link ready. Disconnect threshold {disconnect_timeout:?}"
                                        );
                                        connection_for_watch.mark_link_proven();
                                    }
                                    received_first = true;
                                }
                                if matches!(packet.data, PacketKind::Wind(_)) {
                                    if let Err(e) = wind_sender_clone.send(packet.clone()).await {
                                        debug!("Failed to forward wind packet: {}", e);
                                    }
                                }
                                if let Err(e) = recv_tx_clone.send((packet, None)) {
                                    debug!("Failed to forward coordinator packet: {}", e);
                                }
                            }
                            Err(e) => {
                                error!("Failed to deserialize coordinator packet: {}", e);
                            }
                        }
                    }
                    Ok(Err(e)) => {
                        warn!(
                            "Coordinator connection lost for {:?}: {}",
                            ship_kind_clone, e
                        );
                        let _ = disconnect_tx.send(());
                        break;
                    }
                    Err(_elapsed) => {
                        warn!(
                            "Coordinator heartbeat timeout for {:?} — coordinator unreachable",
                            ship_kind_clone
                        );
                        let _ = disconnect_tx.send(());
                        break;
                    }
                }
            }
        });

        let network_register_packet = Packet {
            header: crate::net::Header {
                source: ShipName::MAX,
                target: CONTROLLER_CLIENT_ID,
            },
            data: PacketKind::JoinRequest {
                kind: Sea::pad_ship_kind_name(&self.kind),
                remove_rules_on_disconnect: self.rm_rules_on_disconnect,
                domain_id: self.domain_id,
                node_mode: self.node_mode,
                disconnect_timeout_ms: self.timing.disconnect_timeout_ms,
            },
        };

        let bytes = to_bytes::<rkyv::rancor::Error>(&network_register_packet)
            .expect("Failed to serialize join request");

        let publisher = self
            .session
            .declare_publisher(&join_key)
            .congestion_control(self.node_mode.congestion_control())
            .reliability(self.node_mode.reliability())
            .wait()
            .expect("Failed to create join publisher");

        debug!("Sending join request to {}", join_key);

        let mut welcome_sub = recv_tx.subscribe();

        // Join requests repeat until a welcome arrives.
        loop {
            publisher
                .put(&*bytes)
                .wait()
                .map_err(|e| anyhow!("Failed to send join request: {}", e))?;

            // The timeout only paces discovery retries.
            let timeout =
                tokio::time::timeout(std::time::Duration::from_millis(500), welcome_sub.recv())
                    .await;

            match timeout {
                Ok(Ok((packet, _))) => {
                    if let PacketKind::Welcome {
                        addr: _,
                        wait_for_ack,
                    } = packet.data
                    {
                        debug!("Received welcome from coordinator");

                        let is_non_compare = match &self.kind {
                            ShipKind::Rat(name) => name != COMPARE_NODE_NAME,
                            _ => true,
                        };
                        if is_non_compare && wait_for_ack {
                            info!("{:?}: waiting for coordinator ready signal", self.kind);
                            Self::wait_for_ack(welcome_sub).await?;
                        }

                        // Registration is complete, so the receive task
                        // switches to timeout mode.
                        let _ = reg_done_tx.send(());
                        return Ok(disconnect_rx);
                    }
                }
                Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(n))) => {
                    warn!("Register receiver lagged by {} messages", n);
                }
                Ok(Err(e)) => {
                    return Err(anyhow!("Channel error during registration: {}", e));
                }
                Err(_) => {
                    debug!("Join request timeout, retrying...");
                }
            }
        }
    }

    async fn wait_for_ack(
        mut coord_sub: tokio::sync::broadcast::Receiver<(Packet, Option<std::net::SocketAddr>)>,
    ) -> anyhow::Result<()> {
        loop {
            match coord_sub.recv().await {
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    warn!("Receiver lagged by {} messages", n);
                    continue;
                }
                Err(e) => {
                    return Err(anyhow!("Could not receive from coordinator: {}", e));
                }
                Ok((packet, _)) => {
                    if matches!(packet.data, PacketKind::Acknowledge) {
                        debug!("Received ack, all clients connected!");
                        return Ok(());
                    }
                }
            }
        }
    }

    pub fn session(&self) -> std::sync::Arc<zenoh::Session> {
        std::sync::Arc::clone(&self.session)
    }

    /// This client's timing policy.
    pub fn timing(&self) -> crate::Timing {
        self.timing
    }

    pub fn domain_id(&self) -> u16 {
        self.domain_id
    }

    /// Build the on-the-wire data payload: id (4) + type (1) + name (64) + body.
    fn build_data_payload(
        id: u32,
        variable_type: VariableType,
        variable_name: &str,
        data: &[u8],
    ) -> zenoh::bytes::ZBytes {
        let id_bytes = id.to_be_bytes();
        let mut padded_name = [0u8; 64];
        let name_bytes = variable_name.as_bytes();
        let len = name_bytes.len().min(64);
        padded_name[..len].copy_from_slice(&name_bytes[..len]);

        let mut payload = Vec::with_capacity(69 + data.len());
        payload.extend_from_slice(&id_bytes);
        payload.push(variable_type.into());
        payload.extend_from_slice(&padded_name);
        payload.extend_from_slice(data);
        zenoh::bytes::ZBytes::from(payload)
    }

    /// One network attempt at delivering `payload` to `data_key`.
    ///
    /// This is a Zenoh query that awaits the receiver's reply, so it costs a
    /// full network round trip. The timeout is set on the query itself so Zenoh
    /// promptly releases its pending state and the payload when the target
    /// queryable has disappeared. Callers holding a lock or a guard across this
    /// call hold it for the whole round trip.
    ///
    /// `congestion_control` must be set explicitly from the target's `Qos`.
    /// Zenoh defaults a *query* to `Block` (`CongestionControl::DEFAULT_REQUEST`)
    /// where a put defaults to `Drop`, and every mode's data path is a query, so
    /// the default makes even `Qos::BestEffort` non-droppable on the wire. A
    /// vanished target then fills its queue with undroppable messages and Zenoh
    /// tears down the whole transport ("Unable to push non droppable network
    /// message. Closing transport!"), coordinator link included.
    async fn try_send_network_once(
        session: &zenoh::Session,
        data_key: &str,
        payload: zenoh::bytes::ZBytes,
        priority: zenoh::qos::Priority,
        congestion_control: zenoh::qos::CongestionControl,
        timeout: std::time::Duration,
    ) -> anyhow::Result<()> {
        let replies = session
            .get(data_key)
            .payload(payload)
            .priority(priority)
            .congestion_control(congestion_control)
            .timeout(timeout)
            .wait()
            .map_err(|e| anyhow::anyhow!("Failed to send data query: {}", e))?;

        let reply = replies
            .recv_async()
            .await
            .map_err(|e| anyhow::anyhow!("data query completed without a reply: {e}"))?;
        reply.result().map(|_| ()).map_err(|e| {
            // The reason is the reply's text payload; the wrapper's debug form
            // buries it in a byte dump.
            let payload = e.payload().to_bytes();
            anyhow::anyhow!(
                "receiver rejected data: {}",
                String::from_utf8_lossy(&payload)
            )
        })
    }

    /// Network-only send (no SHM) for `Qos::BestEffort`: one push, no reply.
    ///
    /// A query would leave a reply channel the sender waits on and abandons once
    /// the target is unreachable; when that target comes back and answers, Zenoh
    /// finds the channel gone and raises `sending on a closed channel` on the
    /// *publisher*. A best-effort peer's comings and goings must never surface
    /// there, so this hands the sample to Zenoh and returns.
    ///
    /// Delivery is therefore unconfirmed. The receiver takes the sample through
    /// the subscriber declared beside its queryable.
    pub async fn send_raw_network(
        session: std::sync::Arc<zenoh::Session>,
        domain_id: u16,
        id: u32,
        data: std::sync::Arc<rkyv::util::AlignedVec>,
        variable_type: VariableType,
        variable_name: String,
        target_ship_name: String,
    ) -> anyhow::Result<()> {
        let data_key = format!(
            "minot/{}/data/{}",
            domain_id,
            sanitize_key(&target_ship_name)
        );
        let payload = Self::build_data_payload(id, variable_type, &variable_name, &data);

        session
            .put(&data_key, payload)
            .priority(zenoh::qos::Priority::Background)
            .congestion_control(Qos::BestEffort.congestion_control())
            .wait()
            .map_err(|e| anyhow::anyhow!("Failed to push best-effort data: {e}"))
    }

    /// Network-only send (no SHM) for `Qos::TryReliable`: retry until the
    /// sample lands or the budget is spent.
    ///
    /// This is the bounded counterpart to [`Self::send_raw_to_other_client`],
    /// which retries without any deadline and so can spin forever against a
    /// target that never answers. Here a delivery that cannot land within
    /// `TRY_RELIABLE_SEND_BUDGET_MS` fails on its own and the caller carries
    /// on. This is the same trade a ROS 2 DDS reliable writer makes when it exhausts
    /// `max_blocking_time`.
    pub async fn send_raw_network_bounded(
        session: std::sync::Arc<zenoh::Session>,
        domain_id: u16,
        id: u32,
        data: std::sync::Arc<rkyv::util::AlignedVec>,
        variable_type: VariableType,
        variable_name: String,
        target_ship_name: String,
    ) -> anyhow::Result<()> {
        let data_key = format!(
            "minot/{}/data/{}",
            domain_id,
            sanitize_key(&target_ship_name)
        );
        let payload = Self::build_data_payload(id, variable_type, &variable_name, &data);

        let deadline = tokio::time::Instant::now()
            + std::time::Duration::from_millis(crate::TRY_RELIABLE_SEND_BUDGET_MS);
        let attempt_timeout =
            std::time::Duration::from_millis(crate::TRY_RELIABLE_ATTEMPT_TIMEOUT_MS);
        let backoff = std::time::Duration::from_millis(crate::TRY_RELIABLE_RETRY_BACKOFF_MS);

        let mut attempts = 0u32;
        let mut last_err;
        loop {
            attempts += 1;
            // Never let one attempt outlive the overall budget.
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            match Self::try_send_network_once(
                &session,
                &data_key,
                payload.clone(),
                zenoh::qos::Priority::Data,
                Qos::TryReliable.congestion_control(),
                attempt_timeout.min(remaining),
            )
            .await
            {
                Ok(()) => {
                    if attempts > 1 {
                        debug!(
                            "Try-reliable send '{}' to '{}' landed on attempt {}",
                            variable_name, target_ship_name, attempts
                        );
                    }
                    return Ok(());
                }
                Err(e) => last_err = e,
            }

            if tokio::time::Instant::now() + backoff >= deadline {
                return Err(anyhow!(
                    "Try-reliable send '{}' to '{}' gave up after {} attempts \
                     ({} ms budget): {last_err}",
                    variable_name,
                    target_ship_name,
                    attempts,
                    crate::TRY_RELIABLE_SEND_BUDGET_MS
                ));
            }
            tokio::time::sleep(backoff).await;
        }
    }

    /// Send raw data to another client via Zenoh query (request-reply).
    pub async fn send_raw_to_other_client(
        &self,
        id: u32,
        data: &[u8],
        variable_type: VariableType,
        variable_name: &str,
        target_ship_name: &str,
    ) -> anyhow::Result<()> {
        let target_key = sanitize_key(target_ship_name);
        let data_key = format!("minot/{}/data/{}", self.domain_id, target_key);

        // Build header: id (4 bytes) + variable_type (1 byte) + name (64 bytes) + data
        let total_len = 69 + data.len();

        let id_bytes = id.to_be_bytes();
        let mut padded_name = [0u8; 64];
        let name_bytes = variable_name.as_bytes();
        let len = name_bytes.len().min(64);
        padded_name[..len].copy_from_slice(&name_bytes[..len]);

        #[cfg(feature = "shm")]
        if total_len >= get_shm_size_threshold() {
            SHM_SEND_ATTEMPTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            // Try SHM transfer with dynamic pool growth
            if let Some(result) = self
                .try_shm_send(
                    total_len,
                    &id_bytes,
                    variable_type,
                    &padded_name,
                    &data,
                    &data_key,
                    id,
                )
                .await
            {
                return result;
            }
            // try_shm_send returned None, so continue on the network path.
            SHM_SEND_FALLBACKS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }

        // Small messages, SHM disabled, or the SHM fallback.
        let mut payload = Vec::with_capacity(total_len);
        payload.extend_from_slice(&id_bytes);
        payload.push(variable_type.into());
        payload.extend_from_slice(&padded_name);
        payload.extend_from_slice(&data);
        // Moving the Vec transfers ownership to Zenoh; passing &Vec would hit
        // the cloning conversion and copy the whole payload.
        let payload = zenoh::bytes::ZBytes::from(payload);

        loop {
            // Spelled out even though Zenoh already defaults a query to
            // `Block`: that implicit default is what hid the same setting on
            // the best-effort path, and `Reliable` is the one mode that wants
            // it.
            let replies = self
                .session
                .get(&data_key)
                .payload(payload.clone())
                .congestion_control(Qos::Reliable.congestion_control())
                .wait()
                .map_err(|e| anyhow!("Failed to send data query: {}", e))?;

            match replies.recv_async().await {
                Ok(reply) => match reply.result() {
                    Ok(_sample) => {
                        #[cfg(feature = "shm")]
                        {
                            NETWORK_SENDS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            NETWORK_SEND_BYTES
                                .fetch_add(total_len as u64, std::sync::atomic::Ordering::Relaxed);
                        }
                        debug!("Sent data id {} to {} (ACK received)", id, data_key);
                        return Ok(());
                    }
                    Err(err) => {
                        let err_payload = err.payload().to_bytes();
                        let err_msg = String::from_utf8_lossy(&err_payload);
                        warn!("Receiver error for id {}: {}", id, err_msg);
                        tokio::task::yield_now().await;
                        continue;
                    }
                },
                Err(_) => {
                    tokio::task::yield_now().await;
                    continue;
                }
            }
        }
    }
}
