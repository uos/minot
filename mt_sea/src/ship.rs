use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;

use crate::net::NetArray;
use anyhow::anyhow;
use log::{debug, error, info, warn};
use rkyv::{api::high::from_bytes, to_bytes, util::AlignedVec};
use tokio::time::{Duration, timeout};
use tokio_util::sync::CancellationToken;
use zenoh::Wait;

use crate::{
    ArchivedMessage, Sendable, ShipKind, VariableType,
    client::Client,
    net::{PacketKind, Qos, sanitize_key},
};

/// Send budget key for asynchronously dispatched sends, so every mode where
/// `Qos::dispatch_is_async` holds. Keyed by target *and* variable: one budget
/// per target would let two variables published in the same frame starve each
/// other, which is invisible on loopback and reproducible over WiFi.
type AsyncSendKey = (String, String);

/// How many sends of one variable may be on the wire to one ship at once.
///
/// Bounded for fairness between variables. Overlapping sends are safe: each is
/// an independent Zenoh query carrying its own reply channel, and the receiver
/// appends to a per-id buffer. On a request/response topic every request shares
/// one variable name, so a bound of 1 would serialize the whole topic.
const MAX_CONCURRENT_SENDS_PER_KEY: usize = 8;

/// How many further sends may wait for a slot before a caller is refused.
/// A refused caller retries in milliseconds; a silently dropped one waits out
/// its own timeout (30 s for an `mt_service` request) for an answer that can
/// never come.
const MAX_QUEUED_SENDS_PER_KEY: usize = 32;

/// Marks a refusal that means "no room right now" on a healthy link, so a
/// caller does not tear down a working connection under load. See
/// [`is_backpressure`].
pub const BACKPRESSURE: &str = "send queue full";

/// True when an error means the link is fine but has no room right now, so the
/// caller should retry.
pub fn is_backpressure(error: &anyhow::Error) -> bool {
    error.to_string().contains(BACKPRESSURE)
}

/// Record a fan-out failure, preferring a hard failure over [`BACKPRESSURE`].
///
/// A fan-out can produce both but only one can be returned, and the order of
/// targets must not decide which: callers shrug off backpressure (`mt_flow`
/// lets its repair timer resend), so reporting it would hide the real failure.
/// `make` is lazy because the message costs a format.
fn remember_failure(slot: &mut Option<anyhow::Error>, make: impl FnOnce() -> anyhow::Error) {
    if slot.as_ref().is_some_and(|held| !is_backpressure(held)) {
        return; // a hard failure outranks anything else
    }
    let error = make();
    if slot.is_none() || !is_backpressure(&error) {
        *slot = Some(error);
    }
}

/// The send budget for one key: permits for what is on the wire, and a count of
/// what is waiting for a permit.
#[derive(Clone, Debug)]
struct SendSlots {
    permits: Arc<tokio::sync::Semaphore>,
    waiting: Arc<std::sync::atomic::AtomicUsize>,
}

impl SendSlots {
    fn new() -> Self {
        Self {
            permits: Arc::new(tokio::sync::Semaphore::new(MAX_CONCURRENT_SENDS_PER_KEY)),
            waiting: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
        }
    }
}

/// Counts a send that is queued for a permit, for as long as it is queued.
struct QueuedSend(Arc<std::sync::atomic::AtomicUsize>);

impl Drop for QueuedSend {
    fn drop(&mut self) {
        self.0.fetch_sub(1, std::sync::atomic::Ordering::Relaxed);
    }
}

#[derive(Debug)]
pub struct NetworkShipImpl {
    pub client: Arc<tokio::sync::Mutex<Client>>,
    /// The coordinator heartbeat channel, held outside the `client` lock that
    /// every send, route lookup and registration contends on. A beat queued
    /// behind data traffic can miss `DISCONNECT_TIMEOUT_MS`, and the
    /// coordinator then drops a client that was healthy and busy.
    heartbeat_send: Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<()>>>>,
    /// Time of the last heartbeat, tracked independently of application
    /// traffic: a busy Wind producer still needs echoes to prove its
    /// coordinator connection to the client-side liveness detector.
    pub last_heartbeat: Arc<tokio::sync::Mutex<Instant>>,
    /// Runtime captured during initialization because `shoot` can be invoked
    /// from an ordinary worker thread with no entered Tokio context.
    runtime_handle: tokio::runtime::Handle,
    /// Cancelled when the coordinator connection is lost.
    pub disconnect: CancellationToken,
    /// (target, variable) pairs that already have one asynchronous delivery in
    /// progress. A new message for a busy pair is dropped before serialization.
    /// other variables to the same target are unaffected.
    async_sends_in_flight: Arc<std::sync::Mutex<HashMap<AsyncSendKey, SendSlots>>>,
    /// Cached routing decisions pushed by the coordinator.
    route_cache: Arc<std::sync::RwLock<HashMap<String, (crate::Action, bool)>>>,
    /// Active peer-monitor tasks: ship_name → abort handle.
    peer_monitor: Arc<std::sync::RwLock<HashMap<String, tokio::task::AbortHandle>>>,
    /// When true, skip the route cache in ask_for_action and always send VariableTaskRequest.
    /// Set for ShipKind::Rat so the coordinator handler runs (TUI catch, comparison loop, etc).
    bypass_cache: bool,
    /// This node's timing policy, used by the heartbeat and peer monitors.
    timing: crate::Timing,
    /// Live coordinator-link state. See `ConnectionState` for reconnect behavior.
    pub connection: Arc<crate::ConnectionState>,
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
        // Identical for every target, so serialize once and keep the aligned
        // allocation alive for the whole fan-out.
        let data_bytes =
            Arc::new(to_bytes::<rkyv::rancor::Error>(data).expect("Could not serialize data"));

        // The fan-out always runs to completion and reports afterwards, so one
        // bad target cannot cut the others off. See [`remember_failure`].
        let mut deferred_error: Option<anyhow::Error> = None;

        for target in targets.iter() {
            let target_ship_name = match &target.kind {
                ShipKind::Rat(name) => name.clone(),
                ShipKind::Wind(name) => name.clone(),
            };

            let target_mode = target.node_mode;
            if target_mode.dispatch_is_async() {
                // Both modes dispatch off the caller's thread so a slow link
                // cannot wedge a publisher's loop. BestEffort tries once,
                // TryReliable retries until its budget is spent.
                let (send_budget, is_reliable_mode) = match target_mode {
                    Qos::TryReliable => (
                        Duration::from_millis(crate::TRY_RELIABLE_SEND_BUDGET_MS)
                            + Duration::from_secs(1),
                        true,
                    ),
                    _ => (Duration::from_secs(5), false),
                };

                let slots = {
                    let mut in_flight = self.async_sends_in_flight.lock().unwrap();
                    in_flight
                        .entry((target_ship_name.clone(), variable_name.to_string()))
                        .or_insert_with(SendSlots::new)
                        .clone()
                };

                // Taken before any permit: `register` holds this lock across a
                // network round trip, and waiting on it while holding a permit
                // would park the whole per-key budget behind that.
                let (session, domain_id) = {
                    let c = self.client.lock().await;
                    (c.session(), c.domain_id())
                };

                let ready = Arc::clone(&slots.permits).try_acquire_owned().ok();
                let queued = match &ready {
                    Some(_) => None,
                    None if !is_reliable_mode => {
                        // A sampled stream prefers the next sample over a late one.
                        debug!(
                            "Dropping {:?} message '{}' for '{}': send already in progress",
                            target_mode, variable_name, target_ship_name
                        );
                        continue;
                    }
                    None => {
                        // TryReliable: nobody else will resend this, so wait
                        // for a slot in the spawned task below, off the
                        // caller's loop.
                        let waiting = slots
                            .waiting
                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        if waiting >= MAX_QUEUED_SENDS_PER_KEY {
                            slots
                                .waiting
                                .fetch_sub(1, std::sync::atomic::Ordering::Relaxed);
                            // Prefixed so the caller can tell congestion from
                            // a dead link and retry immediately. Recorded and
                            // reported after the fan-out, so one congested
                            // subscriber does not stop the sample reaching
                            // everybody else.
                            remember_failure(&mut deferred_error, || {
                                anyhow!(
                                    "{BACKPRESSURE}: too many sends of '{}' already queued for \
                                     '{}' ({} on the wire, {} waiting)",
                                    variable_name,
                                    target_ship_name,
                                    MAX_CONCURRENT_SENDS_PER_KEY,
                                    waiting
                                )
                            });
                            continue;
                        }
                        Some(QueuedSend(Arc::clone(&slots.waiting)))
                    }
                };

                let variable_name = variable_name.to_string();
                let data_bytes = Arc::clone(&data_bytes);
                let permits = Arc::clone(&slots.permits);
                self.runtime_handle.spawn(async move {
                    let _permit = match ready {
                        Some(permit) => permit,
                        None => {
                            // Counted as waiting only while actually waiting.
                            let _queued = queued;
                            match tokio::time::timeout(send_budget, permits.acquire_owned()).await {
                                Ok(Ok(permit)) => permit,
                                _ => {
                                    warn!(
                                        "{:?} send '{}' to '{}' gave up waiting for a send slot",
                                        target_mode, variable_name, target_ship_name
                                    );
                                    return;
                                }
                            }
                        }
                    };
                    let send = async {
                        if is_reliable_mode {
                            Client::send_raw_network_bounded(
                                session,
                                domain_id,
                                id,
                                data_bytes,
                                variable_type,
                                variable_name.clone(),
                                target_ship_name.clone(),
                            )
                            .await
                        } else {
                            Client::send_raw_network(
                                session,
                                domain_id,
                                id,
                                data_bytes,
                                variable_type,
                                variable_name.clone(),
                                target_ship_name.clone(),
                            )
                            .await
                        }
                    };
                    // Backstop only: each send path already bounds itself.
                    match tokio::time::timeout(send_budget, send).await {
                        Ok(Ok(())) => {}
                        // The chain underneath is Zenoh reporting a timed-out
                        // query, and one departed peer turns every publisher
                        // into a source of it, so it stays at debug.
                        Ok(Err(e)) => {
                            warn!(
                                "{:?} send '{}' to '{}' failed",
                                target_mode, variable_name, target_ship_name
                            );
                            debug!(
                                "{:?} send '{}' to '{}' failed: {e:#}",
                                target_mode, variable_name, target_ship_name
                            );
                        }
                        Err(_) => warn!(
                            "{:?} send '{}' to '{}' timed out",
                            target_mode, variable_name, target_ship_name
                        ),
                    }
                });
            } else {
                let client = self.client.lock().await;
                if let Err(error) = client
                    .send_raw_to_other_client(
                        id,
                        data_bytes.as_slice(),
                        variable_type,
                        variable_name,
                        &target_ship_name,
                    )
                    .await
                {
                    remember_failure(&mut deferred_error, || error);
                }
            }
        }

        match deferred_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// Catch the dumped data from the source.
    async fn catch<T: Sendable>(&self, id: u32) -> anyhow::Result<Vec<T>> {
        self.catch_archived::<T>(id)
            .await?
            .into_iter()
            .map(|message| message.deserialize())
            .collect()
    }

    async fn catch_for_variable<T: Sendable>(
        &self,
        id: u32,
        variable_name: &str,
        drop_stale: bool,
    ) -> anyhow::Result<Vec<ArchivedMessage<T>>> {
        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        loop {
            // The route id is only a preference. A coordinator restart derives
            // a new one while publishers still shoot under the old, so entries
            // buffered under any id are taken; requiring `id` would wait out
            // that skew forever.
            let taken = {
                let mut buf_lock = buf.write().unwrap();
                Self::take_for_variable(&mut buf_lock, id, variable_name, drop_stale)
            };

            if let Some(entries) = taken {
                return entries
                    .into_iter()
                    .map(ArchivedMessage::from_aligned_bytes)
                    .collect();
            }

            match update_chan.recv().await {
                // Any arrival may be this variable, whatever id it came under.
                Ok(_) => continue,
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    debug!("catch_for_variable receiver lagged by {n} messages, checking buffer");
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(anyhow!("Update channel closed while waiting for data"));
                }
            }
        }
    }

    async fn catch_archived<T: Sendable>(
        &self,
        id: u32,
    ) -> anyhow::Result<Vec<ArchivedMessage<T>>> {
        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        // The buffer is checked on every iteration because a notification can
        // land between subscribing and waiting on the channel.
        loop {
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                return data_vec
                    .into_iter()
                    .map(|(raw, _, _)| ArchivedMessage::from_aligned_bytes(raw))
                    .collect();
            }

            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        continue;
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
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
        fn to_dyn_str(var_type: VariableType, buf: AlignedVec) -> anyhow::Result<String> {
            Ok(match var_type {
                VariableType::StaticOnly => {
                    return Err(anyhow!(
                        "Received Variable without dynamic type info, could not decode."
                    ));
                }
                VariableType::U8 => {
                    let deserialized = from_bytes::<NetArray<u8>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<u8> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::I32 => {
                    let deserialized = from_bytes::<NetArray<i32>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<i32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F32 => {
                    let deserialized = from_bytes::<NetArray<f32>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<f32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F64 => {
                    let deserialized = from_bytes::<NetArray<f64>, rkyv::rancor::Error>(&buf)?;
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

        // The buffer is checked on every iteration because a notification can
        // land between subscribing and waiting on the channel.
        loop {
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                let mut out_buf = Vec::with_capacity(data_vec.len());
                for (raw, var_type, var_name) in data_vec {
                    out_buf.push((to_dyn_str(var_type, raw)?, var_type, var_name));
                }
                return Ok(out_buf);
            }

            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        continue;
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
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
        // The coordinator pushes route updates, so the cache is authoritative.
        // ShipKind::Rat bypasses it: TUI catch, the comparison loop and
        // RatAction{Shoot} delivery all need the coordinator to see the request.
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

            // Subscribe before sending so the reply cannot arrive first.
            let mut sub = client
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .map(|sub| sub.subscribe())
                .ok_or(anyhow!(
                    "Sender to Coordinator is available but Receiver is not."
                ))?;

            // Release the client lock before sending to avoid a deadlock.
            drop(client);

            sender.send(action_request).await?;

            // The reply exists only in this broadcast stream, whose 256-packet
            // channel drops the oldest on lag. A reply among those is gone for
            // good, so the wait below is bounded: an unbounded one would hang
            // the caller with no error and no timeout.
            let answer = async {
                loop {
                    match sub.recv().await {
                        Ok((packet, _)) => {
                            match packet.data {
                                PacketKind::RatAction {
                                    variable,
                                    action,
                                    lock_until_ack,
                                } => {
                                    // Also cached by the background task; this
                                    // covers the first-call slow path.
                                    self.route_cache
                                        .write()
                                        .unwrap()
                                        .insert(variable.clone(), (action.clone(), lock_until_ack));
                                    if variable == variable_name {
                                        return Ok((action, lock_until_ack));
                                    }
                                }
                                PacketKind::RegistrationError(msg) => {
                                    return Err(anyhow!("{}", msg));
                                }
                                _ => {}
                            }
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            // The answer may have been among the dropped
                            // packets, so keep waiting for the deadline below.
                            log::warn!("ask_for_action receiver lagged by {n} messages");
                            continue;
                        }
                        Err(e) => {
                            return Err(anyhow!(
                                "Could not receive answer for variable question from coordinator: {e}"
                            ));
                        }
                    }
                }
            };

            let deadline = self.timing.registration_timeout();
            match timeout(deadline, answer).await {
                Ok(result) => result,
                Err(_) => Err(anyhow!(
                    "no routing answer for '{variable_name}' from the coordinator within {deadline:?}"
                )),
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
        let (wind_receiver, coord_send) = {
            let client = self.client.lock().await;
            let receiver = std::sync::Arc::clone(&client.wind_receiver);
            let sender = client.coordinator_send.read().unwrap().clone();
            (receiver, sender)
        };

        let sender = coord_send.ok_or(anyhow!("Coordinator send not available"))?;

        let mut receiver = wind_receiver.lock().await;
        match receiver.recv().await {
            Some(packet) => {
                if let PacketKind::Wind(bwd) = packet.data {
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
            .filter(|addr| addr.node_mode.is_monitored())
            .map(|addr| match &addr.kind {
                crate::ShipKind::Rat(name) | crate::ShipKind::Wind(name) => unpad(name),
            })
            .collect(),
        crate::Action::Catch { source, .. } => {
            if !source.node_mode.is_monitored() {
                return HashSet::new();
            }
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

/// Continuously ping a peer and send `PeerDead` to the coordinator after
/// `PEER_DEAD_THRESHOLD` consecutive failures.
///
/// The cancellation token belongs to the *observer* and is only an input here:
/// cancelling it on missed peer heartbeats would make a server kill itself
/// whenever a client disappears.
async fn monitor_peer(
    session: Arc<zenoh::Session>,
    domain_id: u16,
    peer_name: String,
    coord_tx: tokio::sync::mpsc::Sender<crate::net::Packet>,
    disconnect: CancellationToken,
    timing: crate::Timing,
) {
    let key = format!("minot/{}/heartbeat/{}", domain_id, sanitize_key(&peer_name));
    let interval = timing.heartbeat_interval();
    let timeout_dur = timing.disconnect_timeout();
    let mut consecutive_failures = 0u32;

    loop {
        tokio::select! {
            _ = disconnect.cancelled() => return,
            _ = tokio::time::sleep(interval) => {}
        }

        let alive = tokio::time::timeout(timeout_dur, ping_peer(&session, &key))
            .await
            .is_ok_and(|r| r.is_ok());

        if alive {
            consecutive_failures = 0;
        } else {
            consecutive_failures += 1;
            if consecutive_failures >= timing.peer_dead_threshold {
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
                // Best effort: the coordinator may already be gone too.
                coord_tx.send(packet).await.ok();
                return;
            }
        }
    }
}

/// Watch the coordinator link for the lifetime of the node.
///
/// Registration hands back a one-shot that fires when the link drops. A node
/// whose policy allows it then re-registers with backoff and publishes a new
/// generation, and everything holding a publisher or subscriber re-establishes
/// itself against that registration.
///
/// The coordinator supports this: a `JoinRequest` from a name it already knows
/// aborts the stale handler and rejoins, and rules for a node that did not
/// remove them on exit are still there when it comes back.
#[allow(clippy::too_many_arguments)]
fn spawn_connection_supervisor(
    client: Arc<tokio::sync::Mutex<Client>>,
    connection: Arc<crate::ConnectionState>,
    disconnect: CancellationToken,
    disconnect_rx: tokio::sync::oneshot::Receiver<()>,
    policy: crate::ReconnectPolicy,
    timing: crate::Timing,
    kind: ShipKind,
) {
    tokio::spawn(async move {
        let mut link_lost = disconnect_rx;
        loop {
            // Wait for this registration to drop, unless the node is shut down first.
            tokio::select! {
                _ = disconnect.cancelled() => return,
                _ = &mut link_lost => {}
            }

            let (initial_backoff_ms, max_backoff_ms) = match policy {
                crate::ReconnectPolicy::Never => {
                    // Losing the coordinator finishes the node.
                    info!("{kind:?} lost the coordinator and does not reconnect — shutting down");
                    disconnect.cancel();
                    return;
                }
                crate::ReconnectPolicy::Always {
                    initial_backoff_ms,
                    max_backoff_ms,
                } => (initial_backoff_ms, max_backoff_ms),
            };

            connection.mark_disconnected();
            warn!("{kind:?} lost the coordinator — reconnecting");

            let mut backoff = Duration::from_millis(initial_backoff_ms);
            let max_backoff = Duration::from_millis(max_backoff_ms);
            loop {
                tokio::select! {
                    _ = disconnect.cancelled() => return,
                    _ = tokio::time::sleep(backoff) => {}
                }

                // `register` retries the join internally and returns only once
                // welcomed, so bound it here to keep the backoff meaningful
                // when nothing is listening at all.
                let attempt = timeout(timing.registration_timeout(), async {
                    client.lock().await.register().await
                })
                .await;

                match attempt {
                    Ok(Ok(next)) => {
                        link_lost = next;
                        let generation = connection.mark_reconnected();
                        info!("{kind:?} reconnected to the coordinator (generation {generation})");
                        break;
                    }
                    Ok(Err(error)) => {
                        debug!("{kind:?} reconnect attempt failed: {error}");
                    }
                    Err(_elapsed) => {
                        debug!("{kind:?} reconnect attempt timed out");
                    }
                }

                backoff = (backoff * 2).min(max_backoff);
            }
        }
    });
}

impl NetworkShipImpl {
    /// Take this variable's samples, preferring the route `id`, and optionally
    /// keeping only the newest.
    ///
    /// `id` is what the coordinator told this subscriber to expect. A mismatched
    /// id is still accepted because a coordinator restart re-derives the route
    /// under a new id while the publisher may still shoot under the old one.
    fn take_for_variable(
        buf: &mut crate::client::RecvBuffer,
        id: u32,
        variable_name: &str,
        drop_stale: bool,
    ) -> Option<Vec<AlignedVec>> {
        let mut taken = Self::take_variable(buf, id, variable_name).or_else(|| {
            let stale_id = buf.iter().find_map(|(other, entries)| {
                entries
                    .iter()
                    .any(|(_, _, name)| name == variable_name)
                    .then_some(*other)
            })?;
            debug!("No '{variable_name}' under route id {id}, taking stale id {stale_id} instead");
            Self::take_variable(buf, stale_id, variable_name)
        })?;

        if drop_stale && taken.len() > 1 {
            // Arrival-ordered within an id, so the last is the newest.
            debug!(
                "Dropping {} stale '{variable_name}' sample(s) in favour of the newest",
                taken.len() - 1
            );
            taken.drain(..taken.len() - 1);
        }
        Some(taken)
    }

    /// Remove and return this variable's entries under `id`.
    ///
    /// One id can carry several variables, so only this variable's entries are
    /// taken and the id is dropped only once nothing is left under it.
    fn take_variable(
        buf: &mut crate::client::RecvBuffer,
        id: u32,
        variable_name: &str,
    ) -> Option<Vec<AlignedVec>> {
        let entries = buf.get_mut(&id)?;
        // Entries hold whole samples, point clouds among them, so they are
        // drained and moved out by hand. `retain` would only lend its items.
        let mut kept = Vec::with_capacity(entries.len());
        let mut taken = Vec::new();
        for entry in entries.drain(..) {
            if entry.2 == variable_name {
                taken.push(entry.0);
            } else {
                kept.push(entry);
            }
        }
        if kept.is_empty() {
            buf.remove(&id);
        } else {
            *entries = kept;
        }
        (!taken.is_empty()).then_some(taken)
    }

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

    /// Keep the coordinator's handler for this client alive while it is idle.
    ///
    /// The coordinator drops any client that sends nothing for
    /// `DISCONNECT_TIMEOUT_MS` and tears down the task answering its variable
    /// requests, so a ship that only speaks when it has something to say hangs
    /// forever on its next request after a long pause. Every ship must run this
    /// exactly once; `init` leaves it unstarted so the owner of the `Arc`
    /// decides. The task ends when the connection is lost.
    ///
    /// Costs one control message per `HEARTBEAT_INTERVAL_MS`, sent through its
    /// own real-time-priority publisher and bounded channel, so application
    /// traffic cannot queue ahead of it.
    pub fn spawn_heartbeat(self: &std::sync::Arc<Self>) -> tokio::task::JoinHandle<()> {
        // Silence from the coordinator now means something, since this node
        // asks for a reply.
        self.connection.mark_heartbeating();
        let ship = std::sync::Arc::clone(self);
        let disconnect = ship.disconnect.clone();
        tokio::spawn(async move {
            let interval = ship.timing.heartbeat_interval();
            loop {
                tokio::select! {
                    _ = tokio::time::sleep(interval) => {
                        if let Err(e) = ship.send_heartbeat().await {
                            // The node is the only side that can still report
                            // the diverging views.
                            warn!("Failed to send heartbeat: {e}");
                        }
                    }
                    _ = disconnect.cancelled() => return,
                }
            }
        })
    }

    /// Send a heartbeat if enough time has elapsed since the last heartbeat.
    /// Returns Ok(Some(())) if sent, Ok(None) if skipped (too soon), Err on failure.
    pub async fn send_heartbeat(&self) -> anyhow::Result<Option<()>> {
        let elapsed = self.last_heartbeat.lock().await.elapsed();
        if elapsed < self.timing.heartbeat_suppress() {
            return Ok(None);
        }

        let heartbeat_send = self.heartbeat_send.read().unwrap().clone();

        if let Some(sender) = heartbeat_send {
            // Capacity one: a beat only signals current liveness, so one
            // pending is enough and `Full` is the channel working as intended.
            // `Closed` means the forwarder to the network is gone, so the node
            // believes it is alive while the coordinator counts down to
            // dropping it.
            match sender.try_send(()) {
                Ok(()) => {}
                Err(tokio::sync::mpsc::error::TrySendError::Full(())) => {
                    debug!("Heartbeat already pending, skipping this beat");
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(())) => {
                    return Err(anyhow!(
                        "heartbeat path is closed: this node is not reaching the coordinator \
                         and will be dropped by it"
                    ));
                }
            }
            *self.last_heartbeat.lock().await = Instant::now();
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
        }
        Ok(())
    }

    pub async fn init(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        options: crate::NodeOptions,
    ) -> anyhow::Result<Self> {
        Self::init_with_coord_start(
            kind,
            rm_rules_on_disconnect,
            node_mode,
            options,
            |_| async {},
        )
        .await
    }

    /// Like `init`, but on registration timeout calls `start_coord` and retries once.
    ///
    /// The `Client` is created only once and reused for the retry, so no background
    /// tasks are dropped between attempts (avoiding spurious channel-closed errors).
    pub async fn init_with_coord_start<F, Fut>(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        options: crate::NodeOptions,
        start_coord: F,
    ) -> anyhow::Result<Self>
    where
        F: FnOnce(Option<tokio::sync::mpsc::Sender<()>>) -> Fut,
        Fut: std::future::Future<Output = ()>,
    {
        Self::init_with_coord_start_impl(
            kind,
            rm_rules_on_disconnect,
            node_mode,
            options,
            move |tx| async move {
                start_coord(tx).await;
                Ok(())
            },
        )
        .await
    }

    /// Like `init_with_coord_start`, retained as the explicit auto-start entry point.
    ///
    /// Coordinator startup is triggered only by registration timeout. A failure to
    /// open the client session may be unrelated to coordinator availability and is
    /// therefore returned unchanged.
    pub async fn init_with_coord_auto_start<F, Fut>(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        options: crate::NodeOptions,
        start_coord: F,
    ) -> anyhow::Result<Self>
    where
        F: FnOnce(Option<tokio::sync::mpsc::Sender<()>>) -> Fut,
        Fut: std::future::Future<Output = anyhow::Result<()>>,
    {
        Self::init_with_coord_start_impl(
            kind,
            rm_rules_on_disconnect,
            node_mode,
            options,
            start_coord,
        )
        .await
    }

    async fn init_with_coord_start_impl<F, Fut>(
        kind: ShipKind,
        rm_rules_on_disconnect: bool,
        node_mode: Qos,
        options: crate::NodeOptions,
        start_coord: F,
    ) -> anyhow::Result<Self>
    where
        F: FnOnce(Option<tokio::sync::mpsc::Sender<()>>) -> Fut,
        Fut: std::future::Future<Output = anyhow::Result<()>>,
    {
        let timing = options.timing;
        let reconnect = options.reconnect_policy(node_mode);
        let mut start_coord = Some(start_coord);

        // Created before any embedded coordinator starts, so that coordinator
        // can signal this node to shut down through it.
        let (torpedo_tx, mut torpedo_rx) = tokio::sync::mpsc::channel::<()>(1);

        let connection = Arc::new(crate::ConnectionState::new());
        let client = Client::init(
            kind.clone(),
            rm_rules_on_disconnect,
            node_mode,
            timing,
            Arc::clone(&connection),
        )
        .await?;
        let heartbeat_send = client.heartbeat_channel();
        let client = Arc::new(tokio::sync::Mutex::new(client));

        info!("{:?} Registering for network...", &kind);

        let try_register = || async {
            timeout(timing.registration_timeout(), async {
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
                if let Some(start_coord) = start_coord.take() {
                    start_coord(Some(torpedo_tx)).await?;
                }
                // The callback returns only once a coordinator started by this
                // process has installed its join subscriber. When another
                // process owns the coordinator lock, the retry below is itself
                // the readiness wait.
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
        spawn_connection_supervisor(
            Arc::clone(&client),
            Arc::clone(&connection),
            disconnect.clone(),
            disconnect_rx,
            reconnect,
            timing,
            kind.clone(),
        );

        // An embedded coordinator, if one was started, shuts this node down by
        // firing a torpedo.
        let disconnect_torpedo = disconnect.clone();
        tokio::spawn(async move {
            if torpedo_rx.recv().await.is_some() {
                info!("Torpedo received — shutting down");
                disconnect_torpedo.cancel();
            }
        });

        let async_sends_in_flight = Arc::new(std::sync::Mutex::new(HashMap::new()));

        let route_cache = Arc::new(std::sync::RwLock::new(HashMap::<
            String,
            (crate::Action, bool),
        >::new()));
        let peer_monitor = Arc::new(std::sync::RwLock::new(HashMap::<
            String,
            tokio::task::AbortHandle,
        >::new()));

        // Set by the node: every `mt_pubsub`, `mt_scope` and `mt_rat` ship is a
        // `ShipKind::Rat`, so deriving it from the kind would make all of them
        // ask the coordinator for a route on every publish when only the
        // comparison path needs that.
        let bypass_cache = options.bypass_route_cache;

        let ship = Self {
            client,
            heartbeat_send,
            runtime_handle: tokio::runtime::Handle::current(),
            // Far enough in the past that the first heartbeat fires immediately.
            last_heartbeat: Arc::new(tokio::sync::Mutex::new(
                Instant::now() - timing.registration_timeout(),
            )),
            disconnect,
            async_sends_in_flight,
            route_cache,
            peer_monitor,
            bypass_cache,
            timing,
            connection,
        };

        // Background task: take RatAction packets pushed by the coordinator,
        // update route_cache and reconcile the per-peer monitor tasks.
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

            // Registration has run, so the sender is set.
            let mut rx = coord_receive_arc
                .read()
                .unwrap()
                .as_ref()
                .expect("coordinator_receive must be set after registration")
                .subscribe();

            // Peers are tracked per variable and monitored on the union, so a
            // Sail response for one variable does not abort monitors that other
            // variables still need.
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
                                                            timing,
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
                                            // The run is over, so every node stops
                                            // whatever its own QoS. Resilient QoS keeps
                                            // a run alive past one node's death, and
                                            // filtering here would leave nodes
                                            // reconnecting forever against a coordinator
                                            // that has already shut itself down.
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
