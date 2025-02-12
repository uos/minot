use async_trait::async_trait;
use log::error;

pub struct CoordinatorImpl {
    pub sea: crate::net::Sea,
}

#[async_trait]
impl crate::Coordinator for CoordinatorImpl {
    /// Get a channel that only returns the variable from this rat
    async fn rat_action_request_queue(
        &self,
        ship: crate::ShipName,
    ) -> anyhow::Result<tokio::sync::mpsc::Receiver<String>> {
        todo!()
    }
    async fn blow_wind(&self, ship: crate::ShipName, data: crate::WindData) -> anyhow::Result<()> {
        todo!()
    }
    async fn rat_action_send(
        &self,
        ship: crate::ShipName,
        action: crate::Action,
    ) -> anyhow::Result<()> {
        todo!()
    }
}

impl CoordinatorImpl {
    pub async fn new(external_ip: Option<[u8; 4]>) -> Self {
        let sea = crate::net::Sea::init(external_ip).await;

        let clients = std::sync::Arc::new(std::sync::RwLock::new(Vec::new()));
        let mut incoming_clients = sea.network_clients_chan.subscribe();
        tokio::spawn(async move {
            loop {
                match incoming_clients.recv().await {
                    Err(e) => {
                        error!("Coordinator missed clients: {e}");
                    }
                    Ok(client) => {
                        clients.write().unwrap().push(client);
                    }
                }
            }
        });

        Self { sea }
    }
}
