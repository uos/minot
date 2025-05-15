use anyhow::anyhow;
use log::{error, info};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

pub use sea::net::SeaSendableBuffer;
use sea::{ship::NetworkShipImpl, *};

pub struct RatNode {
    name: String,
    ship: Option<Arc<NetworkShipImpl>>,
}

impl RatNode {
    pub async fn create(name: &str) -> anyhow::Result<Self> {
        let ship = sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_string()), None).await?;

        Ok(Self {
            name: name.to_string(),
            ship: Some(Arc::new(ship)),
        })
    }

    pub async fn publish<T>(&self, topic: &str, data: T) -> anyhow::Result<()>
    where
        T: Serialize + Deserialize,
    {
        if let Some(rat_ship) = self.ship.as_ref() {
            match rat_ship.ask_for_action(topic).await {
                Ok((sea::Action::Sail, _)) => {
                    info!("Doing nothing but expected a shoot command {} ", topic);
                    return Ok(());
                }
                Ok((sea::Action::Shoot { target }, _)) => {
                    info!("RatNode {} shoots {} at {:?}", self.name, topic, target);

                    rat_ship
                        .get_cannon()
                        .shoot(&target, data, VariableType::StaticOnly, topic)
                        .await?;

                    info!(
                        "Rat {} finished shooting {} at {:?}",
                        self.name, topic, target
                    );

                    return Ok(());
                }
                Ok((sea::Action::Catch { source: _ }, _)) => {
                    return Err(anyhow!(
                        "Received Catch but we are in a publisher for {} ",
                        topic
                    ));
                }
                Err(e) => {
                    return Err(e);
                }
            };
        }
        Err(anyhow!("ship not initialized"))
    }

    pub async fn subscribe<T>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> anyhow::Result<tokio::sync::mpsc::Receiver<T>>
    where
        T: sea::net::SeaSendableBuffer + 'static,
    {
        if let Some(rat_ship) = self.ship.as_ref() {
            let rat_ship = Arc::clone(&rat_ship);
            let name = self.name.clone();
            let topic = topic.to_owned();
            let (tx, rx) = tokio::sync::mpsc::channel(queue_size);
            tokio::spawn(async move {
                loop {
                    match rat_ship.ask_for_action(&topic).await {
                        Ok((sea::Action::Sail, _)) => {
                            info!("Doing nothing but expected a catch command {} ", topic);
                            continue;
                        }
                        Ok((sea::Action::Shoot { target: _ }, _)) => {
                            error!("Received Shoot but we are in a subscriber for {} ", topic);
                            continue;
                        }
                        Ok((sea::Action::Catch { source }, _)) => {
                            let recv_data = rat_ship.get_cannon().catch::<T>(&source).await;
                            let recv_data = match recv_data {
                                Ok(recv_data) => recv_data,
                                Err(e) => {
                                    error!("Error catching: {e}");
                                    return;
                                }
                            };
                            info!(
                                "RatNode {} finished catching {} from {:?}",
                                name, topic, source
                            );
                            let sent = tx.send(recv_data).await;
                            match sent {
                                Ok(_) => {}
                                Err(e) => {
                                    error!(
                                        "Error sending received buffer out through channel: {e}"
                                    );
                                    return;
                                }
                            }
                        }
                        Err(e) => {
                            error!("Error asking for action: {e}");
                            return; // stopping spawned loop will drop the tx, therefore closing it
                        }
                    };
                }
            });
            return Ok(rx);
        }
        Err(anyhow!("Rat not initialized"))
    }
}
