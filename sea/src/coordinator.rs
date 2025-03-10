use std::collections::{HashMap, HashSet};

use anyhow::anyhow;
use log::{debug, error};

use crate::{
    net::{Packet, PacketKind},
    NetworkShipAddress, ShipName,
};

pub struct ClientInfo {
    pub id: ShipName,
    pub network: NetworkShipAddress,
    pub queue: tokio::sync::broadcast::Sender<String>,
    pub client_port: u16,
    pub sender: tokio::sync::mpsc::Sender<Packet>,
}

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
    pub rat_qs: std::sync::Arc<tokio::sync::RwLock<HashMap<String, ClientInfo>>>,
}

#[async_trait::async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    async fn rat_action_request_queue(
        &self,
        ship: String,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>> {
        debug!("trying to get request queue");
        if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
            return Ok(client_info.queue.subscribe());
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_action_request_queue(ship)).await;
    }

    async fn blow_wind(&self, ship: String, data: crate::WindData) -> anyhow::Result<()> {
        if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::Wind(data),
            };
            return client_info
                .sender
                .send(paket)
                .await
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.blow_wind(ship, data)).await;
    }

    async fn rat_action_send(&self, ship: String, action: crate::ActionPlan) -> anyhow::Result<()> {
        if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
            let action = self.convert_action(&action).await?;
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::RatAction(action),
            };
            return client_info
                .sender
                .send(paket)
                .await
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_action_send(ship, action)).await;
    }
}

impl CoordinatorImpl {
    // wait until all given clients are connected
    pub async fn ensure_clients_connected(
        rats: std::sync::Arc<tokio::sync::RwLock<HashMap<String, ClientInfo>>>,
        clients: HashSet<String>,
    ) {
        loop {
            tokio::time::sleep(tokio::time::Duration::from_millis(30)).await; // Do not spam, let the rats populate in peace... or maybe switch this to just "tokio::task::yield_current().await"
            let mut all_there = true;
            let rat = rats.read().await;
            for client in clients.iter() {
                if rat.get(client).is_none() {
                    all_there = false;
                    break;
                }
            }

            if !all_there {
                continue;
            }

            return;
        }
    }

    async fn convert_action(
        &self,
        human_action: &crate::ActionPlan,
    ) -> anyhow::Result<crate::Action> {
        let action = match human_action {
            crate::ActionPlan::Sail => crate::Action::Sail,
            crate::ActionPlan::Shoot { target } => {
                let mut targets = Vec::with_capacity(target.len());
                for client_name in target.into_iter() {
                    let rat = self.rat_qs.read().await;
                    let client_info = rat
                        .get(client_name)
                        .ok_or_else(|| anyhow!("Unknown client: {}", client_name))?;
                    let mut client_network = client_info.network.clone();
                    client_network.port = client_info.client_port; // use client port for client to client communication
                    targets.push(client_network);
                }

                crate::Action::Shoot { target: targets }
            }
            crate::ActionPlan::Catch { source } => {
                let rat = self.rat_qs.read().await;
                let client_info = rat.get(source).ok_or_else(|| anyhow!("Unknown client."))?;
                crate::Action::Catch {
                    source: client_info.network.clone(),
                }
            }
        };

        Ok(action)
    }

    pub async fn new(external_ip: Option<[u8; 4]>) -> Self {
        let sea = crate::net::Sea::init(external_ip).await;

        let rat_queues = std::sync::Arc::new(tokio::sync::RwLock::new(HashMap::new()));
        let mut incoming_clients = sea.network_clients_chan.subscribe();
        let inner_client_rat_queues = rat_queues.clone();
        tokio::spawn(async move {
            let inner_client_loop_rat_queues = inner_client_rat_queues.clone();
            loop {
                match incoming_clients.recv().await {
                    Err(e) => {
                        error!("Coordinator missed clients: {e}");
                    }
                    Ok(client) => {
                        let (rat_queue_tx, _) = tokio::sync::broadcast::channel(10);
                        let client_info = ClientInfo {
                            id: client.ship,
                            queue: rat_queue_tx.clone(),
                            sender: client.send,
                            network: client.addr_from_coord,
                            client_port: client.other_client_port,
                        };
                        let name = match client.name {
                            crate::ShipKind::Rat(name) => name,
                            crate::ShipKind::Wind(name) => name,
                        };

                        let namea = name.clone();
                        {
                            inner_client_loop_rat_queues
                                .write()
                                .await
                                .insert(name, client_info);
                        }

                        let mut receiver = client.recv.subscribe();
                        tokio::spawn(async move {
                            while let Ok((msg, _)) = receiver.recv().await {
                                match msg.data {
                                    PacketKind::VariableTaskRequest(variable_name) => {
                                        match rat_queue_tx.send(variable_name.clone()) {
                                            Err(e) => {
                                                error!("Could not send to rat queue: {}", e);
                                            }
                                            Ok(_) => {
                                                debug!("sent {}, {:?}", namea, variable_name);
                                            }
                                        }
                                    }
                                    PacketKind::Heartbeat => {
                                        debug!("Got heartbeat.");
                                    }
                                    _ => {}
                                }
                            }
                        });
                    }
                }
            }
        });

        Self {
            sea,
            rat_qs: rat_queues,
        }
    }
    // pub async fn cleanup(&mut self) {
    // self.sea.cleanup().await;
    // }
}
