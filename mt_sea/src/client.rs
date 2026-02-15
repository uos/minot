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
    net::{CONTROLLER_CLIENT_ID, Packet, PacketKind, Sea, get_domain_id, sanitize_key},
};

pub type CoordSender = tokio::sync::broadcast::Sender<(Packet, Option<std::net::SocketAddr>)>;
pub type RecvBuffer = HashMap<u32, Vec<(Vec<u8>, VariableType, String)>>;

#[cfg(feature = "shm")]
/// Within typical Linux /dev/shm limits
const DEFAULT_SHM_BUFFER_SIZE: usize = 7 * 1024 * 1024;

#[cfg(feature = "shm")]
const SHM_SIZE_THRESHOLD: usize = 8 * 1024;

#[cfg(feature = "shm")]
fn is_shm_enabled() -> bool {
    !std::env::var("MINOT_SHM_DISABLED")
        .map(|v| v == "1" || v.to_lowercase() == "true")
        .unwrap_or(false)
}

/// Get SHM buffer size from environment variable, falling back to default
#[cfg(feature = "shm")]
fn get_shm_buffer_size() -> usize {
    std::env::var("MINOT_SHM_SIZE")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(DEFAULT_SHM_BUFFER_SIZE)
}

/// Copy bytes into an aligned buffer for rkyv deserialization
fn align_bytes(bytes: &[u8]) -> AlignedVec {
    let mut aligned = AlignedVec::with_capacity(bytes.len());
    aligned.extend_from_slice(bytes);
    aligned
}

/// SHM state with dynamic resizing capability
#[cfg(feature = "shm")]
struct ShmState {
    provider: std::sync::Arc<ShmProvider<PosixShmProviderBackend>>,
    capacity: usize,
}

pub struct Client {
    pub coordinator_receive: std::sync::Arc<std::sync::RwLock<Option<CoordSender>>>,
    pub coordinator_send:
        std::sync::Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<Packet>>>>,
    pub kind: ShipKind,
    rm_rules_on_disconnect: bool,
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
            .finish_non_exhaustive()
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
    /// Get or create SHM provider, initializing on first call
    #[cfg(feature = "shm")]
    fn get_or_init_shm(&self) -> Option<std::sync::Arc<ShmProvider<PosixShmProviderBackend>>> {
        if !is_shm_enabled() {
            return None;
        }

        // Fast path: already initialized
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

        // Slow path: initialize
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
                    return None;
                }
            }
        }
        state.as_ref().map(|s| s.provider.clone())
    }

    /// Try to grow the SHM pool to accommodate a larger message
    #[cfg(feature = "shm")]
    fn try_grow_shm(
        &self,
        required_size: usize,
    ) -> Option<std::sync::Arc<ShmProvider<PosixShmProviderBackend>>> {
        let mut state = self.shm_state.write().ok()?;

        let current_capacity = state.as_ref().map(|s| s.capacity).unwrap_or(0);

        // Double the required size (like Vec growth strategy)
        let new_capacity = (required_size * 2).max(current_capacity * 2);

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

    /// Get current SHM capacity
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
        // Get or initialize SHM provider
        let mut shm_provider = self.get_or_init_shm()?;

        // Try allocation, growing pool if needed (up to 2 attempts)
        for attempt in 0..2 {
            let shm_result = shm_provider
                .alloc(total_len)
                .with_policy::<BlockOn<GarbageCollect>>()
                .await;

            match shm_result {
                Ok(mut shm_buf) => {
                    // Copy data into SHM buffer
                    shm_buf[0..4].copy_from_slice(id_bytes);
                    shm_buf[4] = variable_type.into();
                    shm_buf[5..69].copy_from_slice(padded_name);
                    shm_buf[69..total_len].copy_from_slice(data);

                    let shm_immut: zenoh::shm::ZShm = shm_buf.into();
                    debug!("Sending {} bytes via SHM to {}", total_len, data_key);

                    // Send with retry loop
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
                    // First attempt failed - try to grow the pool
                    let current_capacity = self.shm_capacity();
                    if total_len > current_capacity {
                        debug!(
                            "Message ({} bytes) exceeds current SHM capacity ({} bytes), growing pool",
                            total_len, current_capacity
                        );
                        if let Some(new_provider) = self.try_grow_shm(total_len) {
                            shm_provider = new_provider;
                            continue; // Retry with larger pool
                        }
                    }
                    // Couldn't grow, fall back to network
                    warn!(
                        "SHM allocation failed for {} bytes, falling back to network transport",
                        total_len
                    );
                    return None;
                }
                Err(_) => {
                    // Second attempt also failed
                    warn!(
                        "SHM allocation failed after pool growth, falling back to network transport"
                    );
                    return None;
                }
            }
        }
        None
    }

    pub async fn init(ship_kind: ShipKind, rm_rules_on_disconnect: bool) -> Self {
        let domain_id = get_domain_id();
        if domain_id > 0 {
            info!("Client using domain ID {}", domain_id);
        }

        // Initialize Zenoh session
        let config = zenoh::Config::default();
        let session = zenoh::open(config)
            .wait()
            .expect("Failed to open Zenoh session");
        let session = std::sync::Arc::new(session);

        let (updated_raw_recv, _) = tokio::sync::broadcast::channel(100);
        let raw_recv_buff: std::sync::Arc<std::sync::RwLock<RecvBuffer>> =
            std::sync::Arc::new(std::sync::RwLock::new(HashMap::new()));

        // Wind channel - use MPSC to buffer wind packets and avoid race conditions
        let (wind_sender, wind_receiver) = tokio::sync::mpsc::channel::<Packet>(100);
        let wind_receiver = std::sync::Arc::new(tokio::sync::Mutex::new(wind_receiver));

        let coord_send_tx = std::sync::Arc::new(std::sync::RwLock::new(None));
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
                        // Get payload bytes, with SHM support when feature is enabled
                        let payload_bytes = match query.payload() {
                            Some(p) => {
                                #[cfg(feature = "shm")]
                                {
                                    // Check if payload is SHM
                                    if let Some(shm_buf) = p.as_shm() {
                                        debug!("Received SHM payload");
                                        shm_buf.to_vec()
                                    } else {
                                        p.to_bytes().to_vec()
                                    }
                                }
                                #[cfg(not(feature = "shm"))]
                                {
                                    p.to_bytes().to_vec()
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

                        // Parse header: id (4 bytes) + variable_type (1 byte) + name (64 bytes) + data
                        if payload_bytes.len() < 69 {
                            error!("Data payload too short");
                            if let Err(e) = query.reply_err("payload too short").wait() {
                                error!("Failed to send error reply: {}", e);
                            }
                            continue;
                        }

                        let id_bytes = [
                            payload_bytes[0],
                            payload_bytes[1],
                            payload_bytes[2],
                            payload_bytes[3],
                        ];
                        let msg_id = u32::from_be_bytes(id_bytes);
                        let variable_type = VariableType::from(payload_bytes[4]);

                        let name_bytes = &payload_bytes[5..69];
                        let var_name = String::from_utf8_lossy(
                            name_bytes.split(|&b| b == 0).next().unwrap_or_default(),
                        )
                        .to_string();

                        let data = payload_bytes[69..].to_vec();

                        {
                            let mut lock = raw_recv_buff_clone.write().unwrap();
                            lock.entry(msg_id)
                                .or_default()
                                .push((data, variable_type, var_name));
                        }

                        if updated_raw_recv_clone.send(msg_id).is_err() {
                            debug!("Data for id {} ready, but no consumers listening", msg_id);
                        }

                        // Reply with ACK to confirm receipt
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

        // Wait for the queryable handler to be running
        let _ = ready_rx.await;
        debug!("Client {} queryable handler ready", ship_name_str);

        Self {
            kind: ship_kind,
            coordinator_send: coord_send_tx,
            coordinator_receive: coord_receive_tx,
            rm_rules_on_disconnect,
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
        }
    }

    /// Register the client to the network
    pub async fn register(&mut self) -> anyhow::Result<tokio::sync::oneshot::Receiver<()>> {
        let ship_name = match &self.kind {
            ShipKind::Rat(name) => name.clone(),
            ShipKind::Wind(name) => name.clone(),
        };
        let ship_name_key = sanitize_key(&ship_name);

        // Key for coordinator -> client messages
        let coord_to_client_key =
            format!("minot/{}/coord/clients/{}", self.domain_id, ship_name_key);
        // Key for client -> coordinator messages
        let client_to_coord_key =
            format!("minot/{}/clients/{}/coord", self.domain_id, ship_name_key);
        let join_key = format!("minot/{}/coord/join", self.domain_id);

        let coord_subscriber = self
            .session
            .declare_subscriber(&coord_to_client_key)
            .wait()
            .expect("Failed to create coordinator subscriber");

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

        // Task to send to coordinator
        let session_for_send = std::sync::Arc::clone(&self.session);
        let client_to_coord_key_owned = client_to_coord_key.clone();
        tokio::spawn(async move {
            let coord_publisher = session_for_send
                .declare_publisher(client_to_coord_key_owned)
                .congestion_control(zenoh::qos::CongestionControl::Block)
                .reliability(zenoh::qos::Reliability::Reliable)
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
        tokio::spawn(async move {
            loop {
                match coord_subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        let aligned = align_bytes(&payload);
                        match from_bytes::<Packet, rkyv::rancor::Error>(&aligned) {
                            Ok(packet) => {
                                // Forward wind packets to dedicated channel (buffered, no race)
                                if matches!(packet.data, PacketKind::Wind(_)) {
                                    if let Err(e) = wind_sender_clone.send(packet.clone()).await {
                                        debug!("Failed to forward wind packet: {}", e);
                                    }
                                }
                                // Also forward to broadcast for other consumers
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
                        warn!(
                            "Coordinator connection lost for {:?}: {}",
                            ship_kind_clone, e
                        );
                        break;
                    }
                }
            }
        });

        // Now send the join request
        let network_register_packet = Packet {
            header: crate::net::Header {
                source: ShipName::MAX,
                target: CONTROLLER_CLIENT_ID,
            },
            data: PacketKind::JoinRequest {
                kind: Sea::pad_ship_kind_name(&self.kind),
                remove_rules_on_disconnect: self.rm_rules_on_disconnect,
                domain_id: self.domain_id,
            },
        };

        let bytes = to_bytes::<rkyv::rancor::Error>(&network_register_packet)
            .expect("Failed to serialize join request");

        // Publisher for join requests
        let publisher = self
            .session
            .declare_publisher(&join_key)
            .congestion_control(zenoh::qos::CongestionControl::Block)
            .reliability(zenoh::qos::Reliability::Reliable)
            .wait()
            .expect("Failed to create join publisher");

        debug!("Sending join request to {}", join_key);

        let mut welcome_sub = recv_tx.subscribe();

        // Keep sending join requests until we get a welcome
        loop {
            publisher
                .put(&*bytes)
                .wait()
                .map_err(|e| anyhow!("Failed to send join request: {}", e))?;

            // Wait for welcome - use a timeout to retry join requests
            // This is acceptable as it's just for retrying discovery, not for correctness
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

                        // For non-compare nodes, wait for coordinator ready signal
                        let is_non_compare = match &self.kind {
                            ShipKind::Rat(name) => name != COMPARE_NODE_NAME,
                            _ => true,
                        };
                        if is_non_compare && wait_for_ack {
                            info!("{:?}: waiting for coordinator ready signal", self.kind);
                            Self::wait_for_ack(welcome_sub).await?;
                        }

                        let (_disconnect_tx, disconnect_rx) = tokio::sync::oneshot::channel();
                        return Ok(disconnect_rx);
                    }
                    // Not a welcome packet, keep waiting
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

    /// Send raw data to another client via Zenoh query (request-reply).
    pub async fn send_raw_to_other_client(
        &self,
        id: u32,
        data: rkyv::util::AlignedVec,
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
        if total_len >= SHM_SIZE_THRESHOLD {
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
            // If try_shm_send returns None, fall through to network
        }

        // Network transfer (small messages, SHM disabled, or SHM fallback)
        let mut payload = Vec::with_capacity(total_len);
        payload.extend_from_slice(&id_bytes);
        payload.push(variable_type.into());
        payload.extend_from_slice(&padded_name);
        payload.extend_from_slice(&data);

        loop {
            let replies = self
                .session
                .get(&data_key)
                .payload(&payload)
                .wait()
                .map_err(|e| anyhow!("Failed to send data query: {}", e))?;

            match replies.recv_async().await {
                Ok(reply) => match reply.result() {
                    Ok(_sample) => {
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
