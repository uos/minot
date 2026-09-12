use anyhow::anyhow;
use log::{debug, error, warn};
use std::{marker::PhantomData, sync::Arc};

use mt_sea::{net::Packet, ship::NetworkShipImpl, *};
use tokio_util::sync::CancellationToken;

pub use mt_sea::{
    ArchivedMessage, Durability, NodeOptions, Qos, ReconnectPolicy, Reliability, Timing,
};

#[derive(Debug, Clone, Copy, Default)]
pub enum CoordMode {
    /// Auto-start an embedded coordinator if none is reachable (default).
    #[default]
    AutoStart,
    /// Start an embedded coordinator before opening the client session.
    ///
    /// Use this when the process is the designated coordinator owner. If another
    /// process already owns the coordinator lock, this node connects to it instead.
    Start,
    /// Fail immediately if no coordinator is reachable. Requires an external coordinator.
    External,
}

#[derive(Debug, Clone)]
pub struct NodeConfig {
    name: String,
    /// Delivery mode. `Reliable` is fatal: if this node dies, the run is
    /// torpedoed. `TryReliable` and `BestEffort` are not, so their loss leaves
    /// the rest of the system running. See `Qos` for the full split.
    mode: Qos,
    /// Controls coordinator startup behavior.
    coord_mode: CoordMode,
    /// Timing and reconnect policy.
    options: NodeOptions,
}

impl NodeConfig {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mode: Qos::Reliable,
            coord_mode: CoordMode::AutoStart,
            options: NodeOptions::default(),
        }
    }

    /// Set timing and reconnect policy in one go.
    pub fn options(mut self, options: NodeOptions) -> Self {
        self.options = options;
        self
    }

    /// Use timing suited to a link that is expected to wobble, and reconnect
    /// when it drops. Shorthand for `.options(NodeOptions::wan())`.
    ///
    /// A `Qos::Reliable` node stays fatal by contract, so pair this with
    /// `Qos::TryReliable`.
    pub fn wan(mut self) -> Self {
        self.options = NodeOptions::wan();
        self
    }

    /// Set how patient this node and the coordinator are with each other.
    pub fn timing(mut self, timing: Timing) -> Self {
        self.options.timing = timing;
        self
    }

    /// Set what happens when the coordinator link drops.
    pub fn reconnect(mut self, policy: ReconnectPolicy) -> Self {
        self.options.reconnect = Some(policy);
        self
    }

    /// Set the node mode (Reliable or BestEffort).
    pub fn mode(mut self, mode: Qos) -> Self {
        self.mode = mode;
        self
    }

    /// Set the coordinator startup behavior.
    pub fn coord_mode(mut self, coord_mode: CoordMode) -> Self {
        self.coord_mode = coord_mode;
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn node_mode(&self) -> Qos {
        self.mode
    }

    pub fn coordinator_mode(&self) -> CoordMode {
        self.coord_mode
    }

    pub fn node_options(&self) -> NodeOptions {
        self.options
    }

    /// Restrict this node, and any embedded coordinator it starts, to this machine.
    pub fn local_only(self, local_only: bool) -> Self {
        mt_sea::network::set_local_only(local_only);
        self
    }
}

/// How long a subscriber waits before retrying a re-registration it still owes,
/// and the ceiling that wait backs off to.
///
/// The coordinator being gone is the ordinary reason to be here, and it can
/// stay gone, so a fixed short delay would mean failed attempts for as long as
/// the node lives.
const REGISTRATION_RETRY_MIN: std::time::Duration = std::time::Duration::from_millis(100);
const REGISTRATION_RETRY_MAX: std::time::Duration = std::time::Duration::from_secs(5);

/// Send a `RegisterShipAtVar` and wait for the coordinator to acknowledge it.
///
/// Coordinator channels are resolved at call time because reconnect replaces
/// them. Returns the sender that was live for this registration.
async fn register_at_var(
    ship: &Arc<NetworkShipImpl>,
    ship_name: &str,
    topic: &str,
    kind: net::RatPubRegisterKind,
    qos: Qos,
) -> anyhow::Result<tokio::sync::mpsc::Sender<Packet>> {
    let (coord_tx, mut coord_rx) = {
        let client = ship.client.lock().await;
        let coord_tx = client
            .coordinator_send
            .read()
            .unwrap()
            .as_ref()
            .ok_or_else(|| anyhow!("not registered with a coordinator"))?
            .clone();
        // Subscribed before the request is sent, so an immediate acknowledgement
        // cannot land before there is anything listening for it.
        let coord_rx = client
            .coordinator_receive
            .read()
            .unwrap()
            .as_ref()
            .ok_or_else(|| anyhow!("not registered with a coordinator"))?
            .subscribe();
        (coord_tx, coord_rx)
    };

    coord_tx
        .send(Packet {
            header: mt_sea::net::Header::default(),
            data: net::PacketKind::RegisterShipAtVar {
                ship: ship_name.to_owned(),
                var: topic.to_owned(),
                kind,
                node_mode: qos,
            },
        })
        .await?;

    loop {
        match coord_rx.recv().await {
            Ok((packet, _)) => match packet.data {
                net::PacketKind::Acknowledge => return Ok(coord_tx),
                net::PacketKind::RegistrationError(message) => return Err(anyhow!(message)),
                _ => continue,
            },
            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
            Err(error) => return Err(anyhow!("coordinator channel closed: {error}")),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Publisher<T: Sendable> {
    topic: String,
    qos: Qos,
    ship: Arc<NetworkShipImpl>,
    name: String,
    /// The connection generation this publisher last registered against. When
    /// the live generation moves past it, the registration belonged to a
    /// connection that no longer exists and has to be redone.
    registered_generation: Arc<std::sync::atomic::AtomicU64>,
    _phantom: PhantomData<T>,
}

impl<T: Sendable> Publisher<T> {
    /// Re-register this publisher if the link was rebuilt since it last did.
    ///
    /// Cheap in the common case: one atomic load that matches.
    async fn ensure_registered(&self) -> anyhow::Result<()> {
        let live = self.ship.connection.generation();
        if self
            .registered_generation
            .load(std::sync::atomic::Ordering::Acquire)
            == live
        {
            return Ok(());
        }
        register_at_var(
            &self.ship,
            &self.name,
            &self.topic,
            net::RatPubRegisterKind::Publish,
            self.qos,
        )
        .await?;
        self.registered_generation
            .store(live, std::sync::atomic::Ordering::Release);
        debug!(
            "Publisher for '{}' re-registered after reconnect (generation {live})",
            self.topic
        );
        Ok(())
    }

    pub async fn publish(&self, data: &T) -> anyhow::Result<()> {
        if self.qos.is_transient_local() {
            self.ship
                .get_cannon()
                .retain(data, VariableType::StaticOnly, &self.topic)
                .await?;
        }
        self.ensure_registered().await?;
        match self.ship.ask_for_action(&self.topic).await {
            Ok((mt_sea::Action::Sail, _)) => {
                debug!("No route for '{}'. Dropping this publish", self.topic);
                Ok(())
            }
            Ok((mt_sea::Action::Shoot { target, id }, _)) => {
                debug!("Publishing to {} at {:?}", self.topic, target);

                self.ship
                    .get_cannon()
                    .shoot(&target, id, data, VariableType::StaticOnly, &self.topic)
                    .await?;

                debug!("Finished publishing {} at {:?}", self.topic, target);

                Ok(())
            }
            Ok((mt_sea::Action::Catch { .. }, _)) => Err(anyhow!(
                "Received Catch but we are in a publisher for {} ",
                self.topic
            )),
            Err(e) => Err(e),
        }
    }
}

#[derive(Debug)]
pub struct Subscriber<T: Sendable> {
    chan: tokio::sync::mpsc::Receiver<ArchivedMessage<T>>,
    /// What it takes to tell the coordinator to stop sending. Held so a
    /// subscriber can be given up without keeping the node around.
    ship: String,
    topic: String,
    /// The sender is resolved here at unsubscribe time because reconnect can
    /// replace the coordinator channels.
    ship_handle: Arc<NetworkShipImpl>,
}

impl<T: Sendable> Subscriber<T> {
    /// Stop delivery of this topic to this node.
    ///
    /// Dropping a subscriber stops local reads. Call this method to stop the
    /// coordinator traffic as well.
    pub async fn unsubscribe(self) -> anyhow::Result<()> {
        let coord_tx = {
            let client = self.ship_handle.client.lock().await;
            let sender = client.coordinator_send.read().unwrap().clone();
            sender.ok_or_else(|| anyhow!("not registered with a coordinator"))?
        };
        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::UnregisterShipAtVar {
                    ship: self.ship.clone(),
                    var: self.topic.clone(),
                    kind: net::RatPubRegisterKind::Subscribe,
                },
            })
            .await?;
        Ok(())
    }

    /// Take the next message if one is already waiting, without blocking.
    ///
    /// `None` means nothing is queued *right now*. Use this when a caller wants
    /// to drain available messages and keep control.
    pub fn try_next(&mut self) -> Option<T> {
        self.chan.try_recv().ok().map(|message| {
            message
                .deserialize()
                .expect("validated message must deserialize")
        })
    }

    /// Receive and deserialize the next message into an owned `T`.
    pub async fn next(&mut self) -> Option<T> {
        self.next_archived().await.map(|message| {
            message
                .deserialize()
                .expect("validated message must deserialize")
        })
    }

    /// Receive the next message as a validated archived view.
    ///
    /// The returned owner keeps its backing buffer alive. Use
    /// [`ArchivedMessage::archived`] to inspect it without creating an owned `T`.
    pub async fn next_archived(&mut self) -> Option<ArchivedMessage<T>> {
        self.chan.recv().await
    }
}

#[derive(Debug, Clone)]
pub struct Node {
    name: String,
    mode: Qos,
    ship: Arc<NetworkShipImpl>,
    /// Cancelled when the coordinator connection is lost.
    shutdown: CancellationToken,
}

impl Node {
    pub async fn create_publisher<T: Sendable>(
        &self,
        topic: String,
        qos: Qos,
    ) -> anyhow::Result<Publisher<T>> {
        register_at_var(
            &self.ship,
            &self.name,
            &topic,
            net::RatPubRegisterKind::Publish,
            qos,
        )
        .await?;

        // A publisher re-registers lazily: `publish` notices a generation
        // change and redoes the registration before sending. Eager
        // re-registration would cost a task per publisher for something that
        // only matters at the next send.
        Ok(Publisher {
            topic,
            qos,
            ship: Arc::clone(&self.ship),
            name: self.name.clone(),
            registered_generation: Arc::new(std::sync::atomic::AtomicU64::new(
                self.ship.connection.generation(),
            )),
            _phantom: PhantomData,
        })
    }

    pub async fn create_subscriber<T: Sendable>(
        &self,
        topic: String,
        queue_size: usize,
        mode: Qos,
    ) -> anyhow::Result<Subscriber<T>> {
        register_at_var(
            &self.ship,
            &self.name,
            &topic,
            net::RatPubRegisterKind::Subscribe,
            mode,
        )
        .await?;

        // The check above covers a publisher that already exists; this task
        // catches a best-effort publisher that registers later.
        let be_error_token = CancellationToken::new();
        {
            let be_error_token_clone = be_error_token.clone();
            let ship_clone = Arc::clone(&self.ship);
            tokio::spawn(async move {
                let mut monitor_rx = {
                    let client = ship_clone.client.lock().await;
                    let lock = client.coordinator_receive.read().unwrap();
                    lock.as_ref().map(|s| s.subscribe())
                };
                if let Some(mut rx) = monitor_rx.take() {
                    loop {
                        match rx.recv().await {
                            Ok((packet, _)) => {
                                if matches!(packet.data, net::PacketKind::RegistrationError(_)) {
                                    be_error_token_clone.cancel();
                                    return;
                                }
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                            Err(_) => return,
                        }
                    }
                }
            });
        }

        let rat_ship = Arc::clone(&self.ship);
        let shutdown = self.ship.disconnect.clone();
        let connection = Arc::clone(&self.ship.connection);
        let mut reconnects = connection.subscribe();
        let ship_name = self.name.clone();
        let subscriber_topic = topic.clone();
        let (tx, rx) = tokio::sync::mpsc::channel(queue_size);

        tokio::spawn(async move {
            // Set when a re-registration after a reconnect did not go through,
            // cleared once one does. The subscription stays alive while it is
            // owed: ending it is invisible to the caller, whose channel simply
            // closes and whose `next()` yields `None` from then on.
            let mut needs_registration = false;
            let mut retry_delay = REGISTRATION_RETRY_MIN;

            loop {
                if tx.is_closed() {
                    return;
                }

                // Nothing else retries a failed re-registration: the next
                // generation only arrives on the next reconnect, which may
                // never come.
                if needs_registration {
                    // Checked here too, since the select below is unreachable
                    // while a registration is owed. A topic that gained a
                    // best-effort publisher during the outage has nothing left
                    // to subscribe to, so the retries must stop.
                    if shutdown.is_cancelled() {
                        return;
                    }
                    if be_error_token.is_cancelled() {
                        error!(
                            "Subscriber for '{topic}' shutting down: topic now has a best-effort publisher"
                        );
                        return;
                    }
                    match register_at_var(
                        &rat_ship,
                        &ship_name,
                        &topic,
                        net::RatPubRegisterKind::Subscribe,
                        mode,
                    )
                    .await
                    {
                        Ok(_) => {
                            needs_registration = false;
                            retry_delay = REGISTRATION_RETRY_MIN;
                            debug!("Subscriber for '{topic}' re-registered on retry");
                        }
                        Err(e) => {
                            debug!(
                                "Subscriber for '{topic}' still cannot re-register, retrying in \
                                 {retry_delay:?}: {e}"
                            );
                            // Backed off, since a coordinator that is simply
                            // gone is the common case here. Cancellable, so a
                            // shutdown does not wait out the longest delay.
                            tokio::select! {
                                _ = shutdown.cancelled() => return,
                                _ = be_error_token.cancelled() => return,
                                _ = tokio::time::sleep(retry_delay) => {}
                            }
                            retry_delay = (retry_delay * 2).min(REGISTRATION_RETRY_MAX);
                            continue;
                        }
                    }
                }

                tokio::select! {
                    // Shutdown and reconnect are checked before doing more work,
                    // so a subscriber never starts a fetch against a dead link.
                    biased;

                    _ = shutdown.cancelled() => {
                        return; // drop tx → closes channel → subber.next() returns None
                    }

                    _ = be_error_token.cancelled() => {
                        error!(
                            "Subscriber for '{topic}' shutting down: topic now has a best-effort publisher"
                        );
                        return; // drop tx → closes channel
                    }

                    // The link came back on a new registration, so whatever the
                    // coordinator knew about this subscription is gone with the
                    // old one and has to be established again. The caller's
                    // `Subscriber` never notices.
                    generation = reconnects.recv() => {
                        match generation {
                            Ok(generation) => {
                                match register_at_var(
                                    &rat_ship,
                                    &ship_name,
                                    &topic,
                                    net::RatPubRegisterKind::Subscribe,
                                    mode,
                                )
                                .await
                                {
                                    Ok(_) => debug!(
                                        "Subscriber for '{topic}' re-registered after reconnect (generation {generation})"
                                    ),
                                    Err(e) => {
                                        // Never fatal. This is where failure is
                                        // most likely: the link was rebuilt an
                                        // instant ago and the coordinator may
                                        // still be tearing down the handler for
                                        // the previous connection. Giving up
                                        // would close the channel and leave
                                        // `next()` at `None` for good.
                                        warn!(
                                            "Subscriber for '{topic}' could not re-register after reconnect, retrying: {e}"
                                        );
                                        needs_registration = true;
                                    }
                                }
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                            Err(_) => return,
                        }
                    }

                    // ask_for_action and catch share a branch so the catch is
                    // cancellable too.
                    result = async {
                        match rat_ship.ask_for_action(&topic).await {
                            Ok((mt_sea::Action::Sail, _)) => {
                                tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                                Ok(None)
                            }
                            Ok((mt_sea::Action::Shoot { .. }, _)) => {
                                error!("Received Shoot but we are in a subscriber for {topic} ");
                                Ok(None)
                            }
                            Ok((mt_sea::Action::Catch { source, id }, _)) => {
                                // The id is only what this subscriber was told
                                // to expect. A coordinator restart re-derives
                                // the route under a new one while a publisher
                                // still shoots under the old, and waiting on the
                                // id alone outlasts that skew for good.
                                //
                                // Whether the backlog may be skipped follows
                                // from the delivery promise. `TryReliable`
                                // dispatches off the caller's thread like
                                // `BestEffort`, but still promises not to drop,
                                // so it takes every sample in arrival order.
                                let recv_data = rat_ship
                                    .get_cannon()
                                    .catch_for_variable::<T>(
                                        id,
                                        &topic,
                                        !mode.expects_reliable_delivery(),
                                    )
                                    .await?;
                                debug!("Finished catching {topic} from {source:?}");
                                Ok(Some(recv_data))
                            }
                            Err(e) => Err(e),
                        }
                    } => {
                        match result {
                            Ok(Some(recv_data)) => {
                                let sender = tx.clone();
                                tokio::spawn(async move {
                                    for rd in recv_data {
                                        if sender.send(rd).await.is_err() {
                                            return;
                                        }
                                    }
                                });
                            }
                            Ok(None) => {}
                            Err(e) => {
                                // Never fatal. A fetch fails for the whole
                                // window around a disconnect, the moment
                                // *before* the supervisor notices included, so
                                // `is_connected()` proves nothing here and
                                // killing the subscription would defeat the
                                // reconnect it is about to get. It ends when the
                                // node shuts down, which for a non-reconnecting
                                // node is when the link drops.
                                debug!("Subscriber for '{topic}' retrying after: {e}");
                                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                            }
                        }
                    }
                }
            }
        });

        Ok(Subscriber {
            chan: rx,
            ship: self.name.to_owned(),
            topic: subscriber_topic,
            ship_handle: Arc::clone(&self.ship),
        })
    }

    pub async fn create(config: NodeConfig) -> anyhow::Result<Self> {
        // A designated owner starts the coordinator without paying discovery
        // time. AutoStart does the same in local-only mode, where the TCP
        // endpoint is a cheap and deterministic existence check.
        let start_before_client = matches!(config.coord_mode, CoordMode::Start)
            || (matches!(config.coord_mode, CoordMode::AutoStart)
                && mt_sea::network::is_local_only()
                && !mt_sea::network::local_router_is_running());
        if start_before_client {
            log::info!("Starting embedded coordinator before client initialization...");
            mt_coord::ensure_default_coordinator_ready(None).await?;
        }

        let rm_rules = config.mode.removes_rules_on_exit();
        let ship = match config.coord_mode {
            CoordMode::External => {
                mt_sea::ship::NetworkShipImpl::init(
                    ShipKind::Rat(config.name.clone()),
                    rm_rules,
                    config.mode,
                    config.options,
                )
                .await?
            }
            CoordMode::AutoStart | CoordMode::Start => {
                mt_sea::ship::NetworkShipImpl::init_with_coord_auto_start(
                    ShipKind::Rat(config.name.clone()),
                    rm_rules,
                    config.mode,
                    config.options,
                    |torpedo_tx| async move {
                        log::info!("No coordinator found, starting embedded coordinator...");
                        mt_coord::ensure_default_coordinator_ready(torpedo_tx)
                            .await
                            .map(|_| ())
                    },
                )
                .await?
            }
        };
        Self::from_ship(config.name, config.mode, ship)
    }

    fn from_ship(
        name: String,
        mode: Qos,
        ship: mt_sea::ship::NetworkShipImpl,
    ) -> anyhow::Result<Self> {
        let shutdown = ship.disconnect.clone();
        let ship = Arc::new(ship);
        ship.spawn_heartbeat();
        Ok(Self {
            name,
            mode,
            ship,
            shutdown,
        })
    }

    /// Returns the QoS mode this node was created with.
    pub fn mode(&self) -> Qos {
        self.mode
    }

    /// Returns a token that is cancelled when this node is finished.
    ///
    /// For a node that reconnects this fires only on a real shutdown, so an
    /// explicit close or a torpedo. Watch [`Node::connection`] for transient
    /// link loss.
    pub fn shutdown_token(&self) -> CancellationToken {
        self.shutdown.clone()
    }

    /// Live state of this node's link to the coordinator.
    ///
    /// Reports whether the link is up and which registration generation is
    /// live. Publishers and subscriber tasks use it to notice that they must
    /// re-register; callers can use it to surface connection status without
    /// polling.
    pub fn connection(&self) -> Arc<mt_sea::ConnectionState> {
        Arc::clone(&self.ship.connection)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{LazyLock, Mutex};

    static NETWORK_FLAG_TEST_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

    #[test]
    fn node_config_defaults_to_network_discovery() {
        let _guard = NETWORK_FLAG_TEST_LOCK.lock().unwrap();
        mt_sea::network::set_local_only(false);
        let config = NodeConfig::new("node");

        assert!(!mt_sea::network::is_local_only());
        assert_eq!(config.name(), "node");
    }

    #[test]
    fn node_config_can_enable_local_only() {
        let _guard = NETWORK_FLAG_TEST_LOCK.lock().unwrap();
        mt_sea::network::set_local_only(false);
        let _config = NodeConfig::new("node").local_only(true);

        assert!(mt_sea::network::is_local_only());
    }
}
