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
    pub client_port: u16,
    pub sender: tokio::sync::mpsc::Sender<Packet>,
}

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
    pub rat_qs: std::sync::Arc<tokio::sync::RwLock<HashMap<String, ClientInfo>>>,
    pub new_rat_note: tokio::sync::broadcast::Sender<String>,
}

#[async_trait::async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    // TODO remove, not used anymore
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

    async fn blow_wind(&self, ship: String, data: Vec<crate::WindData>) -> anyhow::Result<()> {
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
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }
        log::info!("waiting.. ");

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.blow_wind(ship, data)).await;
    }

    async fn rat_action_send(
        &self,
        ship: String,
        action: ActionPlan,
        lock_until_ack: bool,
    ) -> anyhow::Result<()> {
        let client_info = {
            let client_guard = self.rat_qs.read();
            client_guard.await.get(&ship).cloned()
        };

        if let Some(client_info) = client_info {
            let action = self.convert_action(&action).await?;
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::RatAction {
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

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_action_send(ship, action, lock_until_ack)).await;
    }
}

impl CoordinatorImpl {
    pub async fn rat_send(&self, ship: String, kind: PacketKind) -> anyhow::Result<()> {
        if let Some(client_info) = self.rat_qs.read().await.get(&ship) {
            let paket = Packet {
                header: crate::net::Header::default(),
                data: kind,
            };
            return client_info
                .sender
                .send(paket)
                .await
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_send(ship, kind)).await;
    }

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
                // ignore compare node for needed clients
                if client != COMPARE_NODE_NAME && rat.get(client).is_none() {
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
                    let mut client_network = client_info.network.clone();
                    client_network.port = client_info.client_port; // use client port for client to client communication
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
        let mut incoming_clients = sea.network_clients_chan.subscribe();
        let inner_client_rat_queues = rat_queues.clone();
        // let new_rat_note_tx = new_rat_note.clone();
        tokio::spawn(async move {
            let inner_client_loop_rat_queues = inner_client_rat_queues.clone();
            loop {
                match incoming_clients.recv().await {
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!("Coordinator missed {} client events due to channel lag", n);
                        // Continue processing - we may have missed some events but can recover
                        continue;
                    }
                    Err(e) => {
                        error!("Coordinator missed clients: {e}");
                    }
                    Ok(client) => {
                        let (rat_queue_tx, _) = tokio::sync::broadcast::channel(256);
                        let client_info = ClientInfo {
                            id: client.ship,
                            queue: rat_queue_tx.clone(), // TODO not used anymore, rm
                            sender: client.send,
                            network: client.addr_from_coord,
                            client_port: client.other_client_port,
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

                        // new_rat_note_tx.send(name.clone()).unwrap();

                        // let mut receiver = client.recv.subscribe();
                        // tokio::spawn(async move {
                        //     while let Ok((msg, _)) = receiver.recv().await {
                        //         match msg.data {
                        //             PacketKind::VariableTaskRequest(variable_name) => {
                        //                 match rat_queue_tx.send(variable_name.clone()) {
                        //                     Err(e) => {
                        //                         error!("Could not send to rat queue: {}", e);
                        //                     }
                        //                     Ok(_) => {
                        //                         debug!("sent {}, {:?}", &name, variable_name);
                        //                     }
                        //                 }
                        //             }
                        //             PacketKind::Heartbeat => {
                        //                 debug!("Got heartbeat.");
                        //             }
                        //             _ => {
                        //                 debug!(
                        //                     "received a different packet than vartaskreq or hearbeat: {:?}",
                        //                     msg.data
                        //                 );
                        //             }
                        //         }
                        //     }
                        //     info!("closing receiver listener");
                        // });
                    }
                }
            }
        });

        Self {
            sea,
            rat_qs: rat_queues,
            new_rat_note,
        }
    }
    // pub async fn cleanup(&mut self) {
    // self.sea.cleanup().await;
    // }
}
