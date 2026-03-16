use anyhow::{Result, anyhow};
use core::net::SocketAddr;
use log::{debug, info};
use mt_sea::ShipKind;
use mt_sea::{net::Packet, ship::NetworkShipImpl, *};
use once_cell::sync::OnceCell;
use std::collections::HashSet;
use std::sync::Arc;
use std::thread;
use std::time;

use tokio::signal::ctrl_c;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Qos {
    #[default]
    Reliable,
    BestEffort,
}

#[derive(Debug, Clone)]
pub struct ScopeConfig {
    pub name: String,
    pub mode: Qos,
}

// Singleton Objects to make sure this exists only once in the system
static SCOPE: OnceCell<Arc<Mutex<Scope>>> = OnceCell::new();
static COORDCOMMUNICATION: OnceCell<Arc<Mutex<CoordCommunication>>> = OnceCell::new();
static PACKET_ID: OnceCell<Arc<Mutex<i32>>> = OnceCell::new();
static CLIENTS: OnceCell<Arc<Mutex<HashSet<String>>>> = OnceCell::new();
static BEST_EFFORT_CLIENTS: OnceCell<Arc<Mutex<HashSet<String>>>> = OnceCell::new();

#[derive(Debug, Clone)]
pub struct Scope {
    name: String,
    ship: Arc<NetworkShipImpl>,
}

// Save communication channel to minot-coord
struct CoordCommunication {
    coord: (
        tokio::sync::mpsc::Sender<Packet>,
        tokio::sync::broadcast::Receiver<(Packet, Option<SocketAddr>)>,
    ),
}

impl Scope {
    /// Start the Scope. Only once per process due to static OnceCell.
    pub async fn create(config: ScopeConfig) -> anyhow::Result<()> {
        let sea_node_mode = match config.mode {
            Qos::Reliable => mt_sea::Qos::Reliable,
            Qos::BestEffort => mt_sea::Qos::BestEffort,
        };
        let rm_rules_on_disconnect = config.mode == Qos::Reliable;
        let ship = mt_sea::ship::NetworkShipImpl::init(
            ShipKind::Rat(config.name.clone()),
            rm_rules_on_disconnect,
            sea_node_mode,
        )
        .await?;
        debug!("Ship created");

        let scope = Scope {
            name: config.name,
            ship: Arc::new(ship),
        };

        SCOPE
            .set(Arc::new(Mutex::new(scope)))
            .map_err(|_| anyhow!("This scope already exists"))?;

        PACKET_ID
            .set(Arc::new(Mutex::new(0)))
            .map_err(|_| anyhow!("Packet ID already initialized"))?;

        CLIENTS
            .set(Arc::new(Mutex::new(HashSet::new())))
            .map_err(|_| anyhow!("Clients already initialized"))?;

        BEST_EFFORT_CLIENTS
            .set(Arc::new(Mutex::new(HashSet::new())))
            .map_err(|_| anyhow!("Best-effort clients already initialized"))?;

        Scope::init_connection().await?;
        debug!("Coordinator connection established");

        Scope::connect_to_coord().await?;
        debug!("Registered with coordinator");

        Scope::start_scoping().await?;
        debug!("Scoping complete");

        Ok(())
    }

    async fn init_connection() -> anyhow::Result<()> {
        let scope = Scope::get_scope().await?;
        let scope = scope.lock().await;
        let ship = &scope.ship;

        let (coord_tx, coord_rx) = {
            let client = ship.client.lock().await;
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

        let coords = CoordCommunication {
            coord: (coord_tx, coord_rx),
        };

        COORDCOMMUNICATION
            .set(Arc::new(Mutex::new(coords)))
            .map_err(|_| anyhow::anyhow!("COORDCOMMUNICATION already initialized"))?;

        Ok(())
    }

    async fn connect_to_coord() -> anyhow::Result<()> {
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());

        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        tokio::spawn(async move {
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

        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: Scope::get_scope_name().await.unwrap(),
                    var: Scope::get_scope_name().await.unwrap(),
                    kind: net::RatPubRegisterKind::Scope,
                    node_mode: net::Qos::Reliable,
                },
            })
            .await?;

        result_rx.await?;

        Ok(())
    }

    async fn start_scoping() -> anyhow::Result<()> {
        loop {
            tokio::select! {
                _ = ctrl_c() => {
                    debug!("Ctrl-C received. Stopping scoping.");
                    return Ok(())
                }
                res = Scope::threesixty_scoping() => {
                    res?
                }
            }
            let interval = time::Duration::from_millis(1000);
            thread::sleep(interval);
        }
    }

    async fn threesixty_scoping() -> anyhow::Result<()> {
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());
        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        let clients_current = Arc::new(Mutex::new(HashSet::new()));
        let clients_clone = clients_current.clone();

        tokio::spawn(async move {
            let _ = ready_tx.send(());

            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => match packet.data {
                        net::PacketKind::Acknowledge => {
                            let _ = result_tx.send(());
                            return;
                        }
                        net::PacketKind::ClientsHash {
                            mut reliable,
                            best_effort,
                        } => {
                            // Exclude the scope itself from the set it monitors
                            if let Ok(name) = Scope::get_scope_name().await {
                                reliable.remove(&name);
                            }
                            debug!(
                                "[SCOPE] Sonar response — reliable: {:?}, best_effort: {:?}",
                                reliable, best_effort
                            );
                            // Accumulate best-effort names — never remove, so a dying
                            // best-effort node is still recognisable as best-effort even
                            // after the coordinator has already removed it from its own set.
                            if let Some(be_set) = BEST_EFFORT_CLIENTS.get() {
                                let mut be = be_set.lock().await;
                                be.extend(best_effort);
                            }
                            *clients_clone.lock().await = reliable;
                            let _ = result_tx.send(());
                            return;
                        }
                        _ => (),
                    },
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return,
                }
            }
        });

        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::Sonar,
            })
            .await
            .map_err(|_e| anyhow!("Failed to send Sonar packet"))?;

        result_rx
            .await
            .map_err(|_e| anyhow!("Failed to receive Sonar response"))?;

        drop(channels);

        Scope::handle_packet(clients_current.lock().await.clone()).await
    }

    async fn handle_packet(clients_current: HashSet<String>) -> anyhow::Result<()> {
        debug!(
            "[SCOPE] Handling packet #{:?}",
            Scope::get_counter_value().await
        );
        let clients = Scope::get_clients().await?;
        let mut clients = clients.lock().await;

        if clients.is_subset(&clients_current) {
            let new_clients: Vec<_> = clients_current.difference(&clients).cloned().collect();
            if new_clients.is_empty() {
                return Ok(());
            }
            info!("[SCOPE] New clients: {:?}", new_clients);
            for client in new_clients {
                clients.insert(client);
            }
        } else {
            let all_lost: Vec<_> = clients.difference(&clients_current).cloned().collect();

            // Filter out best-effort nodes — losing them should not trigger Torpedo
            let best_effort = BEST_EFFORT_CLIENTS
                .get()
                .and_then(|arc| arc.try_lock().ok());
            let lost_reliable: Vec<_> = match &best_effort {
                Some(be) => all_lost
                    .iter()
                    .filter(|name| !be.contains(*name))
                    .cloned()
                    .collect(),
                None => all_lost.clone(),
            };

            for client in &all_lost {
                clients.remove(client);
            }

            if !lost_reliable.is_empty() {
                info!("[SCOPE] Lost clients, firing Torpedo: {:?}", lost_reliable);
                Scope::send_torpedo(lost_reliable).await?;
            } else {
                debug!(
                    "[SCOPE] Lost best-effort clients, skipping Torpedo: {:?}",
                    all_lost
                );
            }
        }
        Ok(())
    }

    async fn send_torpedo(dead_clients: Vec<String>) -> anyhow::Result<()> {
        debug!("[SCOPE] Sending Torpedo for: {:?}", dead_clients);
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());
        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        tokio::spawn(async move {
            let _ = ready_tx.send(());

            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => {
                        if let net::PacketKind::Acknowledge = packet.data {
                            let _ = result_tx.send(());
                            return;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return,
                }
            }
        });

        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        info!("[SCOPE] Sending Torpedo packet");
        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::Torpedo(dead_clients),
            })
            .await
            .map_err(|_e| anyhow!("Failed to send Torpedo packet"))?;

        result_rx
            .await
            .map_err(|_e| anyhow!("Failed to receive Torpedo acknowledgement"))?;

        drop(channels);
        Ok(())
    }

    async fn get_scope() -> Result<Arc<Mutex<Scope>>> {
        SCOPE
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No scope initialized"))
    }

    async fn get_coord_communication() -> Result<Arc<Mutex<CoordCommunication>>> {
        COORDCOMMUNICATION
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No coordinator communication channel initialized"))
    }

    async fn get_clients() -> Result<Arc<Mutex<HashSet<String>>>> {
        CLIENTS
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No clients set initialized"))
    }

    async fn get_scope_name() -> Result<String> {
        let name = Scope::get_scope().await?.lock().await.name.clone();
        Ok(name)
    }

    async fn get_packet_id() -> Result<Arc<Mutex<i32>>> {
        PACKET_ID
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("Packet ID not initialized"))
    }

    async fn get_counter_value() -> Result<i32> {
        let value = Scope::get_packet_id().await?;
        let mut value = value.lock().await;
        let current = *value;
        *value += 1;
        Ok(current)
    }
}
