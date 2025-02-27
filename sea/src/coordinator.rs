use std::collections::HashMap;

use anyhow::anyhow;
use log::error;

use crate::{
    net::{Packet, PacketKind},
    ShipName,
};

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
    pub senders:
        std::sync::Arc<tokio::sync::RwLock<HashMap<ShipName, tokio::sync::mpsc::Sender<Packet>>>>,
    pub rat_qs: std::sync::Arc<
        tokio::sync::RwLock<HashMap<ShipName, tokio::sync::broadcast::Sender<String>>>,
    >,
}

#[async_trait::async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    async fn rat_action_request_queue(
        &self,
        ship: crate::ShipName,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>> {
        if let Some(receiver) = self.rat_qs.read().await.get(&ship) {
            return Ok(receiver.subscribe());
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_action_request_queue(ship)).await;
    }

    async fn blow_wind(&self, ship: crate::ShipName, data: crate::WindData) -> anyhow::Result<()> {
        if let Some(sender) = self.senders.read().await.get(&ship) {
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::Wind(data),
            };
            return sender
                .send(paket)
                .await
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.blow_wind(ship, data)).await;
    }

    async fn rat_action_send(
        &self,
        ship: crate::ShipName,
        action: crate::Action,
    ) -> anyhow::Result<()> {
        if let Some(sender) = self.senders.read().await.get(&ship) {
            let paket = Packet {
                header: crate::net::Header::default(),
                data: PacketKind::RatAction(action),
            };
            return sender
                .send(paket)
                .await
                .map_err(|e| anyhow!("Error while sending rat action: {e}"));
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        return Box::pin(self.rat_action_send(ship, action)).await;
    }
}

impl CoordinatorImpl {
    pub async fn new(external_ip: Option<[u8; 4]>) -> Self {
        let sea = crate::net::Sea::init(external_ip).await;

        let clients_senders = std::sync::Arc::new(tokio::sync::RwLock::new(HashMap::new()));
        let client_senders_out = clients_senders.clone();
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
                        let (rat_queue_tx, _rat_queue_rx) = tokio::sync::broadcast::channel(10);
                        inner_client_loop_rat_queues
                            .write()
                            .await
                            .insert(client.ship, rat_queue_tx.clone());

                        clients_senders
                            .write()
                            .await
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
            senders: client_senders_out,
        }
    }
}
