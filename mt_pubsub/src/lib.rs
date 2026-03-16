use anyhow::anyhow;
use log::{debug, error};
use std::{marker::PhantomData, sync::Arc};

use mt_sea::{net::Packet, ship::NetworkShipImpl, *};
use tokio_util::sync::CancellationToken;

pub use mt_sea::Qos;

#[derive(Debug, Clone, Copy, Default)]
pub enum CoordMode {
    /// Auto-start an embedded coordinator if none is reachable (default).
    #[default]
    AutoStart,
    /// Fail immediately if no coordinator is reachable. Requires an external coordinator.
    External,
}

#[derive(Debug, Clone)]
pub struct NodeConfig {
    pub name: String,
    /// Whether this node is reliable or best-effort.
    /// Best-effort: if this node crashes, the scope will NOT torpedo other nodes.
    pub mode: Qos,
    /// Controls coordinator startup behavior.
    pub coord_mode: CoordMode,
}

impl NodeConfig {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mode: Qos::Reliable,
            coord_mode: CoordMode::AutoStart,
        }
    }

    /// Set the node mode (Reliable or BestEffort).
    pub fn mode(mut self, mode: Qos) -> Self {
        self.mode = mode;
        self
    }

    /// Set the coordinator mode (AutoStart or External).
    pub fn coord_mode(mut self, coord_mode: CoordMode) -> Self {
        self.coord_mode = coord_mode;
        self
    }
}

#[derive(Debug, Clone)]
pub struct Publisher<T: Sendable> {
    topic: String,
    ship: Arc<NetworkShipImpl>,
    _phantom: PhantomData<T>,
}

impl<T: Sendable> Publisher<T> {
    pub async fn publish(&self, data: &T) -> anyhow::Result<()> {
        match self.ship.ask_for_action(&self.topic).await {
            Ok((mt_sea::Action::Sail, _)) => {
                // debug!("Doing nothing but expected a shoot command {} ", self.topic);
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
    chan: tokio::sync::mpsc::Receiver<T>,
}

impl<T: Sendable> Subscriber<T> {
    pub async fn next(&mut self) -> Option<T> {
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
        let (coord_tx, mut coord_rx) = {
            let client = self.ship.client.lock().await;
            let client_send_lock = client.coordinator_send.read().unwrap();
            let coord_tx = client_send_lock
                .as_ref()
                .expect("Sender does not exist after creation.")
                .clone();

            let client_recv_lock = client.coordinator_receive.read().unwrap();
            let coord_rx = client_recv_lock
                .as_ref()
                .expect("Receiver does not exist after creation")
                .subscribe();
            (coord_tx, coord_rx)
        };

        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        tokio::spawn(async move {
            // Signal that we're ready to receive BEFORE entering the receive loop
            let _ = ready_tx.send(());

            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => {
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            let _ = result_tx.send(());
                            return;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return,
                }
            }
        });

        // Wait for the receiver task to be ready before sending the request
        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        // Request
        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: self.name.to_owned(),
                    var: topic.to_owned(),
                    kind: net::RatPubRegisterKind::Publish,
                    node_mode: qos,
                },
            })
            .await?;

        // Response
        result_rx.await?;

        Ok(Publisher {
            topic,
            ship: Arc::clone(&self.ship),
            _phantom: PhantomData,
        })
    }

    pub async fn create_subscriber<T: Sendable>(
        &self,
        topic: String,
        queue_size: usize,
        mode: Qos,
    ) -> anyhow::Result<Subscriber<T>> {
        let client = self.ship.client.lock().await;
        let coord_tx = {
            let client_send_lock = client.coordinator_send.read().unwrap();
            client_send_lock
                .as_ref()
                .expect("Sender does not exist after creation.")
                .clone()
        };

        let mut coord_rx = {
            let client_recv_lock = client.coordinator_receive.read().unwrap();
            client_recv_lock
                .as_ref()
                .expect("Receiver does not exist after creation")
                .subscribe()
        };

        let (result_tx, result_rx) = tokio::sync::oneshot::channel::<Result<(), String>>();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        tokio::spawn(async move {
            // Signal that we're ready to receive BEFORE entering the receive loop
            let _ = ready_tx.send(());

            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => {
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            let _ = result_tx.send(Ok(()));
                            return;
                        }
                        if let net::PacketKind::RegistrationError(msg) = packet.data {
                            let _ = result_tx.send(Err(msg));
                            return;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return,
                }
            }
        });

        // Wait for the receiver task to be ready before sending the request
        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        // Request
        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: self.name.to_owned(),
                    var: topic.to_owned(),
                    kind: net::RatPubRegisterKind::Subscribe,
                    node_mode: mode,
                },
            })
            .await?;

        // Response
        result_rx.await?.map_err(|e| anyhow!(e))?;

        // Monitor for out-of-band RegistrationError (e.g. a BE publisher registers after us).
        // The registration check above only fires if the publisher was already registered;
        // this persistent task catches the reverse ordering.
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
        let (tx, rx) = tokio::sync::mpsc::channel(queue_size);
        tokio::spawn(async move {
            loop {
                if tx.is_closed() {
                    return;
                }
                tokio::select! {
                    // Put ask_for_action + catch in one branch so catch() is also cancellable.
                    result = async {
                        match rat_ship.ask_for_action(&topic).await {
                            Ok((mt_sea::Action::Sail, _)) => {
                                tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                                Ok(None)
                            }
                            Ok((mt_sea::Action::Shoot { .. }, _)) => {
                                error!("Received Shoot but we are in a subscriber for {} ", &topic);
                                Ok(None)
                            }
                            Ok((mt_sea::Action::Catch { source, id }, _)) => {
                                let recv_data = rat_ship.get_cannon().catch::<T>(id).await?;
                                debug!("Finished catching {} from {:?}", &topic, source);
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
                                error!("Subscriber for '{}' failed: {e}", &topic);
                                return; // drop tx → closes channel
                            }
                        }
                    }
                    _ = be_error_token.cancelled() => {
                        error!(
                            "Subscriber for '{}' shutting down: topic now has a best-effort publisher",
                            &topic
                        );
                        return; // drop tx → closes channel
                    }
                    _ = shutdown.cancelled() => {
                        return; // drop tx → closes channel → subber.next() returns None
                    }
                }
            }
        });

        Ok(Subscriber { chan: rx })
    }

    pub async fn create(config: NodeConfig) -> anyhow::Result<Self> {
        let rm_rules = config.mode == Qos::Reliable;
        let ship = match config.coord_mode {
            CoordMode::External => {
                mt_sea::ship::NetworkShipImpl::init(
                    ShipKind::Rat(config.name.clone()),
                    rm_rules,
                    config.mode,
                )
                .await?
            }
            CoordMode::AutoStart => {
                mt_sea::ship::NetworkShipImpl::init_with_coord_start(
                    ShipKind::Rat(config.name.clone()),
                    rm_rules,
                    config.mode,
                    |torpedo_tx| async move {
                        log::info!("No coordinator found, starting embedded coordinator...");
                        mt_coord::start_default_with_torpedo(torpedo_tx);
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

    /// Returns a token that is cancelled when the coordinator connection is lost.
    pub fn shutdown_token(&self) -> CancellationToken {
        self.shutdown.clone()
    }
}
