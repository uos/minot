use std::collections::HashMap;

use async_trait::async_trait;
use log::error;

use crate::{
    net::{Packet, PacketKind, ShipHandle},
    ShipName,
};

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
    pub senders:
        std::sync::Arc<std::sync::RwLock<HashMap<ShipName, tokio::sync::mpsc::Sender<Packet>>>>,
    pub rat_qs: std::sync::Arc<
        std::sync::RwLock<HashMap<ShipName, tokio::sync::broadcast::Sender<String>>>,
    >,
}

#[async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    async fn rat_action_request_queue(
        &self,
        ship: crate::ShipName,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>> {
        if let Some(receiver) = self.rat_qs.read().unwrap().get(&ship) {
            return Ok(receiver.subscribe());
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return self.rat_action_request_queue(ship).await;
    }

    async fn blow_wind(&self, ship: crate::ShipName, data: crate::WindData) -> anyhow::Result<()> {
        todo!()
    }

    async fn rat_action_send(
        &self,
        ship: crate::ShipName,
        action: crate::Action,
    ) -> anyhow::Result<()> {
        if let Some(sender) = self.senders.read().unwrap().get(&ship) {
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::Disconnect,
            };
            // TODO add packet types for actions and send it via packet
            sender.send(paket).await?;
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return self.rat_action_request_queue(ship).await;
    }
}

impl CoordinatorImpl {
    pub async fn new(external_ip: Option<[u8; 4]>) -> Self {
        let sea = crate::net::Sea::init(external_ip).await;

        let clients_senders = std::sync::Arc::new(std::sync::RwLock::new(HashMap::new()));
        let rat_queues = std::sync::Arc::new(std::sync::RwLock::new(HashMap::new()));
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
                        // TODO check if wind or rat and push differently
                        // clients.write().unwrap().push(client);
                        // create action queue from this rat
                        let (rat_queue_tx, _rat_queue_rx) = tokio::sync::broadcast::channel(10);
                        inner_client_loop_rat_queues
                            .write()
                            .unwrap()
                            .insert(client.ship, rat_queue_tx.clone());

                        clients_senders
                            .write()
                            .unwrap()
                            .insert(client.ship, client.send);

                        let mut receiver = client.recv.subscribe();
                        tokio::spawn(async move {
                            while let Ok(msg) = receiver.recv().await {
                                if let PacketKind::VariableTaskRequest(variable_name) = msg.data {
                                    rat_queue_tx.send(variable_name).unwrap();
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
            senders: clients_senders,
        }
    }
}
