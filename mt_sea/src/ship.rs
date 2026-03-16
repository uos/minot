use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;

use crate::net::NetArray;
use anyhow::anyhow;
use log::{debug, error, info, warn};
use rkyv::{
    Archive, Deserialize,
    api::high::{HighValidator, from_bytes},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    to_bytes,
    util::AlignedVec,
};
use tokio::time::{Duration, timeout};
use tokio_util::sync::CancellationToken;
use zenoh::Wait;

use crate::{
    COORDINATOR_STARTUP_WAIT_MS, HEARTBEAT_SUPPRESS_MS, PEER_DEAD_THRESHOLD,
    REGISTRATION_TIMEOUT_MS, Sendable, ShipKind, VariableType,
    client::Client,
    net::{PacketKind, Qos, sanitize_key},
};

/// Copy bytes into an aligned buffer for rkyv deserialization
fn align_bytes(bytes: &[u8]) -> AlignedVec {
    let mut aligned = AlignedVec::with_capacity(bytes.len());
    aligned.extend_from_slice(bytes);
    aligned
}

struct BeSendTask {
    session: std::sync::Arc<zenoh::Session>,
    domain_id: u16,
    id: u32,
    data: AlignedVec,
    variable_type: VariableType,
    variable_name: String,
    target_ship_name: String,
}

#[derive(Debug)]
pub struct NetworkShipImpl {
    pub client: Arc<tokio::sync::Mutex<Client>>,
    pub last_send: Arc<tokio::sync::Mutex<Instant>>,
    /// Cancelled when the coordinator connection is lost.
    pub disconnect: CancellationToken,
    /// Sync sender for fire-and-forget BE deliveries (background task spawned at init).
    be_send_tx: tokio::sync::mpsc::UnboundedSender<BeSendTask>,
    /// Cached routing decisions pushed by the coordinator.
    route_cache: Arc<std::sync::RwLock<HashMap<String, (crate::Action, bool)>>>,
    /// Active peer-monitor tasks: ship_name → abort handle.
    peer_monitor: Arc<std::sync::RwLock<HashMap<String, tokio::task::AbortHandle>>>,
    /// When true, skip the route cache in ask_for_action and always send VariableTaskRequest.
    /// Set for ShipKind::Rat so the coordinator handler runs (TUI catch, comparison loop, etc).
    bypass_cache: bool,
}

#[async_trait::async_trait]
impl crate::Cannon for NetworkShipImpl {
    async fn shoot<'b, T: Sendable>(
        &self,
        targets: &'b [crate::NetworkShipAddress],
        id: u32,
        data: &T,
        variable_type: VariableType,
        variable_name: &str,
    ) -> anyhow::Result<()> {
        for target in targets.iter() {
            let data_bytes =
                to_bytes::<rkyv::rancor::Error>(data).expect("Could not serialize data");
            let target_ship_name = match &target.kind {
                ShipKind::Rat(name) => name.clone(),
                ShipKind::Wind(name) => name.clone(),
            };

            if target.node_mode == Qos::BestEffort {
                let (session, domain_id) = {
                    let c = self.client.lock().await;
                    (c.session(), c.domain_id())
                };
                // UnboundedSender::send is sync — safe to call from any context.
                let _ = self.be_send_tx.send(BeSendTask {
                    session,
                    domain_id,
                    id,
                    data: data_bytes,
                    variable_type,
                    variable_name: variable_name.to_string(),
                    target_ship_name,
                });
            } else {
                let client = self.client.lock().await;
                client
                    .send_raw_to_other_client(
                        id,
                        data_bytes,
                        variable_type,
                        variable_name,
                        &target_ship_name,
                    )
                    .await?;
            }
        }
        *self.last_send.lock().await = Instant::now();
        Ok(())
    }

    /// Catch the dumped data from the source.
    async fn catch<T>(&self, id: u32) -> anyhow::Result<Vec<T>>
    where
        T: Send,
        T: Archive,
        T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    {
        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        // Loop until we get data for our id
        // We must check the buffer on EVERY iteration because notifications
        // might have been sent before we started waiting on the channel
        loop {
            // Check if data is already in buffer
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                // Data found - deserialize and return
                let mut out_buf = Vec::with_capacity(data_vec.len());
                for (raw, _, _) in data_vec {
                    let aligned = align_bytes(&raw);
                    let mat = from_bytes::<T, rkyv::rancor::Error>(&aligned)
                        .expect("Could not decode data to T");
                    out_buf.push(mat);
                }
                return Ok(out_buf);
            }

            // No data yet - wait for notification
            // Use recv() which blocks until a message arrives
            // If we miss a notification, the next iteration will check the buffer again
            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        // Our data might be ready, loop back to check buffer
                        continue;
                    }
                    // Not our id, keep waiting
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    // We missed some messages - that's fine, just check the buffer
                    debug!("Catch receiver lagged by {} messages, checking buffer", n);
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(anyhow!("Update channel closed while waiting for data"));
                }
            }
        }
    }

    async fn catch_dyn(&self, id: u32) -> anyhow::Result<Vec<(String, VariableType, String)>> {
        fn to_dyn_str(var_type: VariableType, buf: Vec<u8>) -> anyhow::Result<String> {
            let aligned = align_bytes(&buf);
            Ok(match var_type {
                VariableType::StaticOnly => {
                    return Err(anyhow!(
                        "Received Variable without dynamic type info, could not decode."
                    ));
                }
                VariableType::U8 => {
                    let deserialized = from_bytes::<NetArray<u8>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<u8> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::I32 => {
                    let deserialized = from_bytes::<NetArray<i32>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<i32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F32 => {
                    let deserialized = from_bytes::<NetArray<f32>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<f32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F64 => {
                    let deserialized = from_bytes::<NetArray<f64>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<f64> = deserialized.into();
                    format!("{:?}", mat)
                }
            })
        }

        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        // Loop until we get data for our id
        // We must check the buffer on EVERY iteration because notifications
        // might have been sent before we started waiting on the channel
        loop {
            // Check if data is already in buffer
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                // Data found - convert and return
                let mut out_buf = Vec::with_capacity(data_vec.len());
                for (raw, var_type, var_name) in data_vec {
                    out_buf.push((to_dyn_str(var_type, raw)?, var_type, var_name));
                }
                return Ok(out_buf);
            }

            // No data yet - wait for notification
            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        // Our data might be ready, loop back to check buffer
                        continue;
                    }
                    // Not our id, keep waiting
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    // We missed some messages - that's fine, just check the buffer
                    debug!(
                        "Catch_dyn receiver lagged by {} messages, checking buffer",
                        n
                    );
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(anyhow!("Update channel closed while waiting for data"));
                }
            }
        }
    }
}

#[async_trait::async_trait]
impl crate::Ship for NetworkShipImpl {
    async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<(crate::Action, bool)> {
        // Fast path — use cached route if available (coordinator pushes updates proactively).
        // Bypassed for ShipKind::Rat so VariableTaskRequest always reaches the coordinator
        // (required for TUI catch, comparison loop, and RatAction{Shoot} delivery).
        if !self.bypass_cache {
            if let Some(cached) = self.route_cache.read().unwrap().get(variable_name).cloned() {
                debug!("ask_for_action: cache hit for {}", variable_name);
                return Ok(cached);
            }
        }

        debug!(
            "ask_for_action: cache miss for {}, asking coordinator",
            variable_name
        );

        let client = self.client.lock().await;
        let coord_send = client.coordinator_send.read().unwrap().clone();
        if let Some(sender) = coord_send {
            let action_request = crate::net::Packet {
                header: crate::net::Header::default(),
                data: PacketKind::VariableTaskRequest(variable_name.to_string()),
            };

            // Subscribe BEFORE sending request to avoid race condition
            let mut sub = client
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .map(|sub| sub.subscribe())
                .ok_or(anyhow!(
                    "Sender to Coordinator is available but Receiver is not."
                ))?;

            // Release client lock before sending to avoid deadlock
            drop(client);

            // Send the request
            sender.send(action_request).await?;

            // Wait for the response matching our variable
            loop {
                match sub.recv().await {
                    Ok((packet, _)) => {
                        match packet.data {
                            PacketKind::RatAction {
                                variable,
                                action,
                                lock_until_ack,
                            } => {
                                // Cache every RatAction we see (background task also does this,
                                // but caching here covers the first-call slow path).
                                self.route_cache
                                    .write()
                                    .unwrap()
                                    .insert(variable.clone(), (action.clone(), lock_until_ack));
                                if variable == variable_name {
                                    return Ok((action, lock_until_ack));
                                }
                                // Wrong variable, keep waiting
                            }
                            PacketKind::RegistrationError(msg) => {
                                return Err(anyhow!("{}", msg));
                            }
                            _ => {
                                // Not a RatAction or RegistrationError, keep waiting
                            }
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!(
                            "ask_for_action receiver lagged by {} messages, continuing",
                            n
                        );
                        continue;
                    }
                    Err(e) => {
                        return Err(anyhow!(
                            "Could not receive answer for variable question from coordinator: {e}"
                        ));
                    }
                }
            }
        } else {
            drop(client);
            tokio::task::yield_now().await;
            return self.ask_for_action(variable_name).await;
        }
    }

    fn get_cannon(&self) -> &impl crate::Cannon {
        self
    }

    async fn wait_for_wind(&self) -> anyhow::Result<Vec<crate::WindData>> {
        // Get the wind receiver and coordinator sender from client
        let (wind_receiver, coord_send) = {
            let client = self.client.lock().await;
            let receiver = std::sync::Arc::clone(&client.wind_receiver);
            let sender = client.coordinator_send.read().unwrap().clone();
            (receiver, sender)
        };

        let sender = coord_send.ok_or(anyhow!("Coordinator send not available"))?;

        // Lock the wind receiver and wait for the next packet
        let mut receiver = wind_receiver.lock().await;
        match receiver.recv().await {
            Some(packet) => {
                if let PacketKind::Wind(bwd) = packet.data {
                    // Send ack to coordinator
                    sender
                        .send(crate::net::Packet {
                            header: crate::net::Header::default(),
                            data: PacketKind::Acknowledge,
                        })
                        .await
                        .map_err(|e| anyhow!("Failed to send wind ack: {}", e))?;
                    return Ok(bwd.into_iter().map(|wa| wa.data).collect::<Vec<_>>());
                } else {
                    return Err(anyhow!("Expected Wind packet but got something else"));
                }
            }
            None => {
                return Err(anyhow!("Wind channel closed"));
            }
        }
    }
}

/// Extract peer ship names that must be monitored given a route action.
/// Names in `NetworkShipAddress` are padded by the coordinator (64-char `#` suffix).
/// Strip that padding so the name matches the heartbeat queryable key declared by the peer.
fn extract_peers(action: &crate::Action) -> HashSet<String> {
    fn unpad(name: &str) -> String {
        name.trim_end_matches('#').to_string()
    }
    match action {
        crate::Action::Sail => HashSet::new(),
        crate::Action::Shoot { target, .. } => target
            .iter()
            .map(|addr| match &addr.kind {
                crate::ShipKind::Rat(name) | crate::ShipKind::Wind(name) => unpad(name),
            })
            .collect(),
        crate::Action::Catch { source, .. } => {
            let name = match &source.kind {
                crate::ShipKind::Rat(name) | crate::ShipKind::Wind(name) => unpad(name),
            };
            std::iter::once(name).collect()
        }
    }
}

/// Send a Zenoh query to a peer's heartbeat key and return Ok if a reply arrives.
async fn ping_peer(session: &zenoh::Session, key: &str) -> anyhow::Result<()> {
    let replies = session
        .get(key)
        .priority(zenoh::qos::Priority::DataHigh)
        .wait()
        .map_err(|e| anyhow!("ping_peer: failed to send query: {}", e))?;
    replies
        .recv_async()
        .await
        .map_err(|_| anyhow!("ping_peer: no reply received"))?
        .result()
        .map(|_| ())
        .map_err(|_| anyhow!("ping_peer: peer replied with error"))
}

/// Continuously ping a peer; send `PeerDead` to the coordinator after
/// `PEER_DEAD_THRESHOLD` consecutive failures, then cancel the local disconnect
/// token so this node shuts down even if the coordinator is already gone.
async fn monitor_peer(
    session: Arc<zenoh::Session>,
    domain_id: u16,
    peer_name: String,
    coord_tx: tokio::sync::mpsc::Sender<crate::net::Packet>,
    disconnect: CancellationToken,
) {
    let key = format!("minot/{}/heartbeat/{}", domain_id, sanitize_key(&peer_name));
    let interval = Duration::from_millis(crate::HEARTBEAT_INTERVAL_MS);
    let timeout_dur = Duration::from_millis(crate::DISCONNECT_TIMEOUT_MS);
    let mut consecutive_failures = 0u32;

    loop {
        tokio::time::sleep(interval).await;

        let alive = tokio::time::timeout(timeout_dur, ping_peer(&session, &key))
            .await
            .is_ok_and(|r| r.is_ok());

        if alive {
            consecutive_failures = 0;
        } else {
            consecutive_failures += 1;
            if consecutive_failures >= PEER_DEAD_THRESHOLD {
                warn!(
                    "Peer {} declared dead after {} consecutive ping failures",
                    peer_name, consecutive_failures
                );
                let packet = crate::net::Packet {
                    header: crate::net::Header::default(),
                    data: PacketKind::PeerDead {
                        ship: peer_name.clone(),
                    },
                };
                // Best-effort notify coordinator; may fail if it is also gone.
                coord_tx.send(packet).await.ok();
                // Cancel local disconnect so this node shuts down regardless of
                // whether the coordinator is still alive to send a torpedo back.
                disconnect.cancel();
                return;
            }
        }
    }
}

impl NetworkShipImpl {
    #[allow(dead_code)]
    async fn spawn_recursive_rejoin_task(
        disconnect_handle: tokio::sync::oneshot::Receiver<()>,
        client: Arc<tokio::sync::Mutex<Client>>,
    ) {
        match disconnect_handle.await {
            Err(e) => {
                error!("Error receiving disconnect signal: {e}");
            }
            Ok(_) => {
                let res = { client.lock().await.register().await };
                match res {
                    Err(e) => {
                        error!("Could not register after dropped connection: {e}");
                    }
                    Ok(recv) => {
                        info!("Reconnected");
                        Box::pin(Self::spawn_recursive_rejoin_task(recv, client)).await;
                    }
                }
            }
        }
    }

    /// Send a heartbeat to the coordinator if enough time has elapsed since the last send.
    /// Returns Ok(Some(())) if sent, Ok(None) if skipped (too soon), Err on failure.
    pub async fn send_heartbeat(&self) -> anyhow::Result<Option<()>> {
        let elapsed = self.last_send.lock().await.elapsed();
        if elapsed < Duration::from_millis(HEARTBEAT_SUPPRESS_MS) {
            return Ok(None);
        }

        let coord_send = {
            let client = self.client.lock().await;
            client.coordinator_send.read().unwrap().clone()
        };

        if let Some(sender) = coord_send {
            let packet = crate::net::Packet {
                header: crate::net::Header::default(),
                data: PacketKind::Heartbeat,
            };
            sender.send(packet).await?;
            *self.last_send.lock().await = Instant::now();
            Ok(Some(()))
        } else {
            Ok(None)
        }
    }

    pub async fn send_wind(&self, messages: Vec<crate::net::WindAt>) -> anyhow::Result<()> {
        let sender = {
            let client = self.client.lock().await;
            client.coordinator_send.read().unwrap().clone()
        };
        if let Some(sender) = sender {
            sender
                .send(crate::net::Packet {
                    header: crate::net::Header::default(),
                    data: crate::net::PacketKind::Wind(messages),
                })
                .await?;
            *self.last_send.lock().await = Instant::now();
        }
        Ok(())
    }

    pub async fn init(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
    ) -> anyhow::Result<Self> {
        Self::init_with_coord_start(kind, rm_rules_on_disconnect, node_mode, |_| async {}).await
    }

    /// Like `init`, but on registration timeout calls `start_coord` and retries once.
    ///
    /// The `Client` is created only once and reused for the retry, so no background
    /// tasks are dropped between attempts (avoiding spurious channel-closed errors).
    pub async fn init_with_coord_start<F, Fut>(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        start_coord: F,
    ) -> anyhow::Result<Self>
    where
        F: FnOnce(Option<tokio::sync::mpsc::Sender<()>>) -> Fut,
        Fut: std::future::Future<Output = ()>,
    {
        let client = Client::init(kind.clone(), rm_rules_on_disconnect, node_mode).await?;
        let client = Arc::new(tokio::sync::Mutex::new(client));

        info!("{:?} Registering for network...", &kind);

        // Create torpedo channel before potentially starting an embedded coordinator so that
        // the coordinator can signal this node to shut down via the torpedo mechanism.
        let (torpedo_tx, mut torpedo_rx) = tokio::sync::mpsc::channel::<()>(1);

        let try_register = || async {
            timeout(Duration::from_millis(REGISTRATION_TIMEOUT_MS), async {
                client.lock().await.register().await
            })
            .await
        };

        let disconnect_rx = match try_register().await {
            Ok(Ok(handle)) => {
                info!("{:?} Registered.", &kind);
                handle
            }
            Ok(Err(e)) => return Err(e),
            Err(_elapsed) => {
                // No coordinator found — call the provided startup function and retry
                start_coord(Some(torpedo_tx)).await;
                tokio::time::sleep(Duration::from_millis(COORDINATOR_STARTUP_WAIT_MS)).await;
                match try_register().await {
                    Ok(Ok(handle)) => {
                        info!("{:?} Registered.", &kind);
                        handle
                    }
                    Ok(Err(e)) => return Err(e),
                    Err(_elapsed) => {
                        return Err(anyhow::anyhow!(
                            "{:?} Registration timed out — no coordinator reachable",
                            &kind
                        ));
                    }
                }
            }
        };

        let disconnect = CancellationToken::new();
        let disconnect_cancel = disconnect.clone();
        tokio::spawn(async move {
            let _ = disconnect_rx.await;
            disconnect_cancel.cancel();
        });

        // Cancel disconnect when the embedded coordinator fires a torpedo (if one was started).
        let disconnect_torpedo = disconnect.clone();
        tokio::spawn(async move {
            if torpedo_rx.recv().await.is_some() {
                info!("Torpedo received — shutting down");
                disconnect_torpedo.cancel();
            }
        });

        // Spawn background worker for BE fire-and-forget sends.
        // Spawned here where the runtime is guaranteed; shoot() just pushes to the channel.
        let (be_send_tx, mut be_send_rx) = tokio::sync::mpsc::unbounded_channel::<BeSendTask>();
        tokio::spawn(async move {
            while let Some(task) = be_send_rx.recv().await {
                tokio::spawn(async move {
                    let _ = tokio::time::timeout(
                        Duration::from_secs(5),
                        Client::send_raw_network(
                            task.session,
                            task.domain_id,
                            task.id,
                            task.data,
                            task.variable_type,
                            task.variable_name,
                            task.target_ship_name,
                        ),
                    )
                    .await;
                });
            }
        });

        let route_cache = Arc::new(std::sync::RwLock::new(HashMap::<
            String,
            (crate::Action, bool),
        >::new()));
        let peer_monitor = Arc::new(std::sync::RwLock::new(HashMap::<
            String,
            tokio::task::AbortHandle,
        >::new()));

        let bypass_cache = matches!(kind, ShipKind::Rat(_));

        let ship = Self {
            client,
            // Initialize far enough in the past so the first heartbeat fires immediately
            last_send: Arc::new(tokio::sync::Mutex::new(
                Instant::now() - Duration::from_millis(REGISTRATION_TIMEOUT_MS),
            )),
            disconnect,
            be_send_tx,
            route_cache,
            peer_monitor,
            bypass_cache,
        };

        // Background task: listen for RatAction packets pushed by the coordinator,
        // update route_cache, and reconcile per-peer monitor tasks.
        {
            let coord_receive_arc = {
                let c = ship.client.lock().await;
                Arc::clone(&c.coordinator_receive)
            };
            let coord_send_arc = {
                let c = ship.client.lock().await;
                Arc::clone(&c.coordinator_send)
            };
            let (session, domain_id) = {
                let c = ship.client.lock().await;
                (c.session(), c.domain_id())
            };

            let route_cache_bg = Arc::clone(&ship.route_cache);
            let peer_monitor_bg = Arc::clone(&ship.peer_monitor);
            let shutdown = ship.disconnect.clone();

            // subscribe() after registration guarantees the sender is set
            let mut rx = coord_receive_arc
                .read()
                .unwrap()
                .as_ref()
                .expect("coordinator_receive must be set after registration")
                .subscribe();

            // Per-variable peer sets: the required global peer set is their union.
            // This prevents a Sail response for one variable (e.g. /nothing_here)
            // from aborting monitors that are still needed by other variables.
            let mut per_var_peers: HashMap<String, HashSet<String>> = HashMap::new();

            tokio::spawn(async move {
                loop {
                    tokio::select! {
                        _ = shutdown.cancelled() => {
                            let mut monitors = peer_monitor_bg.write().unwrap();
                            for (_, handle) in monitors.drain() {
                                handle.abort();
                            }
                            return;
                        }
                        result = rx.recv() => {
                            match result {
                                Ok((packet, _)) => {
                                    match packet.data {
                                        PacketKind::RatAction {
                                            variable,
                                            action,
                                            lock_until_ack,
                                        } => {
                                            route_cache_bg.write().unwrap()
                                                .insert(variable.clone(), (action.clone(), lock_until_ack));

                                            // Update this variable's peer set and recompute the union.
                                            per_var_peers.insert(variable.clone(), extract_peers(&action));
                                            let required: HashSet<String> = per_var_peers
                                                .values()
                                                .flat_map(|s| s.iter().cloned())
                                                .collect();

                                            let current_peers: HashSet<String> = {
                                                peer_monitor_bg
                                                    .read()
                                                    .unwrap()
                                                    .keys()
                                                    .cloned()
                                                    .collect()
                                            };

                                            // Abort monitors for peers no longer needed by any variable
                                            {
                                                let to_remove: Vec<String> = current_peers
                                                    .iter()
                                                    .filter(|p| !required.contains(*p))
                                                    .cloned()
                                                    .collect();
                                                let mut monitors = peer_monitor_bg.write().unwrap();
                                                for peer in to_remove {
                                                    if let Some(handle) = monitors.remove(&peer) {
                                                        handle.abort();
                                                    }
                                                }
                                            }

                                            // Spawn monitors for newly required peers
                                            for peer in &required {
                                                if !current_peers.contains(peer) {
                                                    let coord_sender =
                                                        coord_send_arc.read().unwrap().clone();
                                                    if let Some(sender) = coord_sender {
                                                        let task = tokio::spawn(monitor_peer(
                                                            Arc::clone(&session),
                                                            domain_id,
                                                            peer.clone(),
                                                            sender,
                                                            shutdown.clone(),
                                                        ));
                                                        peer_monitor_bg
                                                            .write()
                                                            .unwrap()
                                                            .insert(peer.clone(), task.abort_handle());
                                                    }
                                                }
                                            }
                                        }
                                        PacketKind::Torpedo(dead_clients) => {
                                            info!(
                                                "Torpedo received from coordinator for {:?} — shutting down",
                                                dead_clients
                                            );
                                            shutdown.cancel();
                                        }
                                        _ => {}
                                    }
                                }
                                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                                    continue;
                                }
                                Err(_) => {
                                    // Channel closed — abort all peer monitors and exit
                                    let mut monitors = peer_monitor_bg.write().unwrap();
                                    for (_, handle) in monitors.drain() {
                                        handle.abort();
                                    }
                                    return;
                                }
                            }
                        }
                    }
                }
            });
        }

        Ok(ship)
    }
}
