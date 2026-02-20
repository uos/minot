use anyhow::anyhow;
use log::{debug, error};
use std::{marker::PhantomData, sync::Arc};

use mt_sea::{net::Packet, ship::NetworkShipImpl, *};

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

#[derive(Debug)]
pub struct Node {
    name: String,
    ship: Arc<NetworkShipImpl>,
}

impl Node {
    pub async fn create_publisher<T: Sendable>(
        &self,
        topic: String,
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
                    kind: net::RatPubRegisterKind::Subscribe,
                },
            })
            .await?;

        // Response
        result_rx.await?;

        let rat_ship = Arc::clone(&self.ship);
        let (tx, rx) = tokio::sync::mpsc::channel(queue_size);
        tokio::spawn(async move {
            loop {
                match rat_ship.ask_for_action(&topic).await {
                    Ok((mt_sea::Action::Sail, _)) => {
                        // debug!("Doing nothing but expected a catch command {} ", &topic);
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    Ok((mt_sea::Action::Shoot { .. }, _)) => {
                        error!("Received Shoot but we are in a subscriber for {} ", &topic);
                        continue;
                    }
                    Ok((mt_sea::Action::Catch { source, id }, _)) => {
                        let recv_data = rat_ship.get_cannon().catch::<T>(id).await;
                        let recv_data = match recv_data {
                            Ok(recv_data) => recv_data,
                            Err(e) => {
                                error!("Error catching: {e}");
                                return;
                            }
                        };
                        debug!("Finished catching {} from {:?}", &topic, source);
                        let sender = tx.clone();
                        tokio::spawn(async move {
                            for rd in recv_data {
                                let sent = sender.send(rd).await;
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
                        });
                    }
                    Err(e) => {
                        error!("Error asking for action: {e}");
                        return; // stopping spawned loop will drop the tx, therefore closing it
                    }
                };
            }
        });

        Ok(Subscriber { chan: rx })
    }

    pub async fn create(name: String) -> anyhow::Result<Self> {
        let ship =
            mt_sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_owned()), true).await?;

        Ok(Self {
            name,
            ship: Arc::new(ship),
        })
    }
}
