use anyhow::anyhow;
use log::{debug, error};
use rkyv::{
    Archive, Deserialize, api::high::HighValidator, bytecheck::CheckBytes, de::Pool,
    rancor::Strategy,
};
use std::{marker::PhantomData, sync::Arc};

use sea::{net::Packet, ship::NetworkShipImpl, *};

#[derive(Debug, Clone)]
pub struct Publisher<T: Sendable> {
    topic: String,
    ship: Arc<NetworkShipImpl>,
    _phantom: PhantomData<T>,
}

impl<T: Sendable> Publisher<T> {
    pub async fn publish(&self, data: &T) -> anyhow::Result<()> {
        match self.ship.ask_for_action(&self.topic).await {
            Ok((sea::Action::Sail, _)) => {
                // debug!("Doing nothing but expected a shoot command {} ", self.topic);
                return Ok(());
            }
            Ok((sea::Action::Shoot { target, id }, _)) => {
                debug!("Publishing to {} at {:?}", self.topic, target);

                self.ship
                    .get_cannon()
                    .shoot(&target, id, data, VariableType::StaticOnly, &self.topic)
                    .await?;

                debug!("Finished publishing {} at {:?}", self.topic, target);

                return Ok(());
            }
            Ok((sea::Action::Catch { .. }, _)) => {
                return Err(anyhow!(
                    "Received Catch but we are in a publisher for {} ",
                    self.topic
                ));
            }
            Err(e) => {
                return Err(e);
            }
        };
    }
}

#[derive(Debug)]
pub struct Subscriber<T>
where
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
{
    chan: tokio::sync::mpsc::Receiver<T>,
}

impl<'t, T> Subscriber<T>
where
    T: Send + Sync + 'static,
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
{
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

        let (tx, rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            while !coord_rx
                .recv()
                .await
                .map(|(packet, _)| matches!(packet.data, net::PacketKind::Acknowledge))
                .unwrap()
            {}

            tx.send(()).unwrap();
        });

        // // Request
        coord_tx
            .send(Packet {
                header: sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: self.name.to_owned(),
                    var: topic.to_owned(),
                    kind: net::RatPubRegisterKind::Publish,
                },
            })
            .await?;

        // Response
        rx.await?;

        Ok(Publisher {
            topic,
            ship: Arc::clone(&self.ship),
            _phantom: PhantomData::default(),
        })
    }

    pub async fn create_subscriber<T>(
        &self,
        topic: String,
        queue_size: usize,
    ) -> anyhow::Result<Subscriber<T>>
    where
        T: Send + Sync + 'static,
        T: Archive,
        T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    {
        let client = self.ship.client.lock().await;
        let client_send_lock = client.coordinator_send.read().unwrap();
        let coord_tx = client_send_lock
            .as_ref()
            .expect("Sender does not exist after creation.");

        let client_recv_lock = client.coordinator_receive.read().unwrap();
        let mut coord_rx = client_recv_lock
            .as_ref()
            .expect("Receiver does not exist after creation")
            .subscribe();

        let (tx, rx) = tokio::sync::oneshot::channel();
        tokio::spawn(async move {
            while !coord_rx
                .recv()
                .await
                .map(|(packet, _)| matches!(packet.data, net::PacketKind::Acknowledge))
                .unwrap()
            {}

            tx.send(()).unwrap();
        });

        // Request
        coord_tx
            .send(Packet {
                header: sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: self.name.to_owned(),
                    var: topic.to_owned(),
                    kind: net::RatPubRegisterKind::Subscribe,
                },
            })
            .await?;

        // Response
        rx.await?;

        let rat_ship = Arc::clone(&self.ship);
        let (tx, rx) = tokio::sync::mpsc::channel(queue_size);
        tokio::spawn(async move {
            loop {
                match rat_ship.ask_for_action(&topic).await {
                    Ok((sea::Action::Sail, _)) => {
                        // debug!("Doing nothing but expected a catch command {} ", &topic);
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    Ok((sea::Action::Shoot { .. }, _)) => {
                        error!("Received Shoot but we are in a subscriber for {} ", &topic);
                        continue;
                    }
                    Ok((sea::Action::Catch { source, id }, _)) => {
                        let recv_data = rat_ship.get_cannon().catch::<T>(id).await;
                        let recv_data = match recv_data {
                            Ok(recv_data) => recv_data,
                            Err(e) => {
                                error!("Error catching: {e}");
                                return;
                            }
                        };
                        debug!("Finished catching {} from {:?}", &topic, source);
                        let sent = tx.send(recv_data).await;
                        match sent {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Error sending received buffer out through channel: {e}");
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

        Ok(Subscriber { chan: rx })
    }

    pub async fn create(name: String) -> anyhow::Result<Self> {
        let ship =
            sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_owned()), None, true).await?;

        Ok(Self {
            name,
            ship: Arc::new(ship),
        })
    }
}
