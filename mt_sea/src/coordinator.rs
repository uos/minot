use std::collections::{HashMap, HashSet};

use anyhow::anyhow;
use log::{debug, error};
use mt_net::{ActionPlan, COMPARE_NODE_NAME};

use crate::{
    NetworkShipAddress, ShipName,
    net::{Packet, PacketKind, WindAt},
};

#[derive(Clone, Debug)]
pub struct ClientInfo {
    pub id: ShipName,
    pub network: NetworkShipAddress,
    pub queue: tokio::sync::broadcast::Sender<String>,
    pub sender: tokio::sync::mpsc::Sender<Packet>,
}

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
    pub rat_qs: std::sync::Arc<tokio::sync::RwLock<HashMap<String, ClientInfo>>>,
    pub new_rat_note: tokio::sync::broadcast::Sender<String>,
    pub new_client_notify: tokio::sync::broadcast::Sender<String>,
}

#[async_trait::async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    async fn rat_action_request_queue(
        &self,
        ship: String,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>> {
        debug!("trying to get request queue");
        // Subscribe BEFORE checking the map to avoid race condition
        // where client is added between check and subscribe
        let mut notify = self.new_client_notify.subscribe();
        loop {
            if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
                return Ok(client_info.queue.subscribe());
            }

            // Wait for a new client notification
            loop {
                match notify.recv().await {
                    Ok(name) if name == ship => break,
                    Ok(_) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Re-check the map in case we missed notifications
                        break;
                    }
                    Err(_) => return Err(anyhow!("Client notification channel closed")),
                }
            }
        }
    }

    async fn blow_wind(&self, ship: String, data: Vec<crate::WindData>) -> anyhow::Result<()> {
        // Subscribe BEFORE checking the map to avoid race condition
        let mut notify = self.new_client_notify.subscribe();
        loop {
            if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
                let paket = Packet {
                    header: crate::net::Header::default(),
                    data: PacketKind::Wind(
                        data.into_iter()
                            .map(|wd| WindAt {
                                data: wd,
                                at_var: None,
                            })
                            .collect::<Vec<_>>(),
                    ),
                };
                return client_info
                    .sender
                    .send(paket)
                    .await
                    .map_err(|e| anyhow!("Error while sending wind: {e}"));
            }

            // Wait for client to connect
            loop {
                match notify.recv().await {
                    Ok(name) if name == ship => break,
                    Ok(_) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Re-check the map in case we missed notifications
                        break;
                    }
                    Err(_) => return Err(anyhow!("Client notification channel closed")),
                }
            }
        }
    }

    async fn rat_action_send(
        &self,
        ship: String,
        variable: String,
        action: ActionPlan,
        lock_until_ack: bool,
    ) -> anyhow::Result<()> {
        // Subscribe BEFORE checking the map to avoid race condition
        let mut notify = self.new_client_notify.subscribe();
        loop {
            let client_info = {
                let client_guard = self.rat_qs.read().await;
                client_guard.get(&ship).cloned()
            };

            if let Some(client_info) = client_info {
                let action = self.convert_action(&action).await?;
                let paket = Packet {
                    header: crate::net::Header::default(),
                    data: PacketKind::RatAction {
                        variable,
                        action,
                        lock_until_ack,
                    },
                };
                return client_info
                    .sender
                    .send(paket)
                    .await
                    .map_err(|e| anyhow!("Error while sending rat action: {e}"));
            }

            // Wait for client to connect
            loop {
                match notify.recv().await {
                    Ok(name) if name == ship => break,
                    Ok(_) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Re-check the map in case we missed notifications
                        break;
                    }
                    Err(_) => return Err(anyhow!("Client notification channel closed")),
                }
            }
        }
    }
}

impl CoordinatorImpl {
    pub async fn rat_send(&self, ship: String, kind: PacketKind) -> anyhow::Result<()> {
        // Subscribe BEFORE checking the map to avoid race condition
        let mut notify = self.new_client_notify.subscribe();
        loop {
            if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
                let paket = Packet {
                    header: crate::net::Header::default(),
                    data: kind,
                };
                return client_info
                    .sender
                    .send(paket)
                    .await
                    .map_err(|e| anyhow!("Error while sending: {e}"));
            }

            // Wait for client to connect
            loop {
                match notify.recv().await {
                    Ok(name) if name == ship => break,
                    Ok(_) => continue,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                        // Re-check the map in case we missed notifications
                        break;
                    }
                    Err(_) => return Err(anyhow!("Client notification channel closed")),
                }
            }
        }
    }

    /// Wait until all given clients are connected (deterministic, no timing)
    pub async fn ensure_clients_connected(
        rats: std::sync::Arc<tokio::sync::RwLock<HashMap<String, ClientInfo>>>,
        clients: HashSet<String>,
        notify: tokio::sync::broadcast::Sender<String>,
    ) {
        let mut remaining: HashSet<String> = clients
            .into_iter()
            .filter(|c| c != COMPARE_NODE_NAME)
            .collect();

        // First check what's already connected
        {
            let rat = rats.read().await;
            remaining.retain(|client| !rat.contains_key(client));
        }

        if remaining.is_empty() {
            return;
        }

        // Wait for remaining clients
        let mut sub = notify.subscribe();
        while !remaining.is_empty() {
            match sub.recv().await {
                Ok(name) => {
                    remaining.remove(&name);
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                    // Re-check current state
                    let rat = rats.read().await;
                    remaining.retain(|client| !rat.contains_key(client));
                }
                Err(_) => {
                    // Channel closed - check if all connected
                    let rat = rats.read().await;
                    remaining.retain(|client| !rat.contains_key(client));
                    if remaining.is_empty() {
                        return;
                    }
                    log::warn!(
                        "Client notification channel closed with {} clients remaining",
                        remaining.len()
                    );
                    return;
                }
            }
        }
    }

    async fn convert_action(&self, human_action: &ActionPlan) -> anyhow::Result<crate::Action> {
        let action = match human_action {
            ActionPlan::Sail => crate::Action::Sail,
            ActionPlan::Shoot { target, id } => {
                let mut targets = Vec::with_capacity(target.len());
                for client_name in target.iter() {
                    let rat = self.rat_qs.read().await;
                    let client_info = rat
                        .get(client_name)
                        .ok_or_else(|| anyhow!("Unknown client: {}", client_name))?;
                    // In Zenoh, we use the network address which contains the ship kind with name
                    let client_network = client_info.network.clone();
                    targets.push(client_network);
                }

                crate::Action::Shoot {
                    target: targets,
                    id: *id,
                }
            }
            ActionPlan::Catch { source, id } => {
                let rat = self.rat_qs.read().await;
                let client_info = rat.get(source).ok_or_else(|| anyhow!("Unknown client."))?;
                crate::Action::Catch {
                    source: client_info.network.clone(),
                    id: *id,
                }
            }
        };

        Ok(action)
    }

    pub async fn new(
        external_ip: Option<[u8; 4]>,
        clients_wait_for_ack: std::sync::Arc<std::sync::RwLock<bool>>,
    ) -> Self {
        let sea = crate::net::Sea::init(external_ip, clients_wait_for_ack).await;

        let rat_queues = std::sync::Arc::new(tokio::sync::RwLock::new(HashMap::new()));
        let (new_rat_note, _) = tokio::sync::broadcast::channel::<String>(128);
        let (new_client_notify, _) = tokio::sync::broadcast::channel::<String>(128);

        let mut incoming_clients = sea.network_clients_chan.subscribe();
        let inner_client_rat_queues = rat_queues.clone();
        let notify_tx = new_client_notify.clone();

        tokio::spawn(async move {
            let inner_client_loop_rat_queues = inner_client_rat_queues.clone();
            loop {
                match incoming_clients.recv().await {
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!("Coordinator missed {} client events due to channel lag", n);
                        continue;
                    }
                    Err(e) => {
                        error!("Coordinator missed clients: {e}");
                    }
                    Ok(client) => {
                        let (rat_queue_tx, _) = tokio::sync::broadcast::channel(256);
                        let client_info = ClientInfo {
                            id: client.ship,
                            queue: rat_queue_tx,
                            sender: client.send,
                            network: client.addr_from_coord,
                        };
                        let name = match client.name {
                            crate::ShipKind::Rat(name) => name,
                            crate::ShipKind::Wind(name) => name,
                        };

                        {
                            inner_client_loop_rat_queues
                                .write()
                                .await
                                .insert(name.clone(), client_info);
                        }

                        // Notify waiters that a new client connected
                        let _ = notify_tx.send(name.clone());
                        debug!("Client {} connected", name);
                    }
                }
            }
        });

        Self {
            sea,
            rat_qs: rat_queues,
            new_rat_note,
            new_client_notify,
        }
    }
}
