use std::{net::IpAddr, str::FromStr, sync::Arc};

use anyhow::anyhow;
use log::{error, info};

use crate::{
    net::{Client, PacketKind, SeaSendableBuffer},
    ShipKind,
};

pub struct NetworkShipImpl {
    client: Arc<tokio::sync::Mutex<Client>>,
}

#[async_trait::async_trait]
impl crate::Cannon for NetworkShipImpl {
    async fn shoot(
        &self,
        targets: &Vec<crate::NetworkShipAddress>,
        data: impl crate::net::SeaSendableBuffer,
    ) -> anyhow::Result<()> {
        let client = self.client.lock().await;
        for target in targets.iter() {
            let data = data.clone().to_packet();
            let ip_addr = &IpAddr::from_str(&format!(
                "{}.{}.{}.{}",
                target.ip[0], target.ip[1], target.ip[2], target.ip[3]
            ))
            .unwrap();
            client
                .send_raw_to_other_client(ip_addr, target.port, data)
                .await?;
        }
        Ok(())
    }

    /// Catch the dumped data from the source.
    async fn catch<T: SeaSendableBuffer>(
        &self,
        _target: &crate::NetworkShipAddress, // TODO rm target, since port is fixed?
    ) -> anyhow::Result<T> {
        let client = self.client.lock().await;
        let data = client.recv_raw_from_other_client().await?;
        let mat = T::set_from_packet(data).expect("Could not decode data to Matrix");
        Ok(mat)
    }
}

#[async_trait::async_trait]
impl crate::Ship for NetworkShipImpl {
    async fn wait_for_action(&self, kind: crate::ShipKind) -> anyhow::Result<crate::Action> {
        let client = self.client.lock().await;
        if let Some(receiver) = client.coordinator_receive.as_ref() {
            let mut sub = receiver.subscribe();
            while let Ok(packet) = sub.recv().await {
                if let PacketKind::RatAction(action) = packet.data {
                    return Ok(action);
                }
            }
            return Err(anyhow!(
                "Could not receive action while Channel was subscribed."
            ));
        } else {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            return self.wait_for_action(kind).await;
        }
    }

    async fn ask_for_action(
        &self,
        kind: crate::ShipKind,
        variable_name: &str,
    ) -> anyhow::Result<crate::Action> {
        let client = self.client.lock().await;
        if let Some(sender) = client.coordinator_send.as_ref() {
            let action_request = crate::net::Packet {
                header: crate::net::Header::default(),
                data: PacketKind::VariableTaskRequest(variable_name.to_string()),
            };
            sender.send(action_request).await?;
            return self.wait_for_action(kind).await;
        } else {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            return self.ask_for_action(kind, &variable_name).await;
        }
    }

    fn get_cannon(&self) -> &impl crate::Cannon {
        self
    }
}

impl NetworkShipImpl {
    async fn spawn_recursive_rejoin_task(
        disconnect_handle: tokio::sync::oneshot::Receiver<()>,
        client: Arc<tokio::sync::Mutex<crate::net::Client>>,
    ) {
        match disconnect_handle.await {
            Err(e) => {
                error!("Error receiving disconnect signal: {e}");
            }
            Ok(_) => {
                let res = { client.lock().await.register().await };
                match res {
                    Err(e) => {
                        error!("Could not register after dropped connection: {e}");
                    }
                    Ok(recv) => {
                        info!("Reconnected");
                        Box::pin(Self::spawn_recursive_rejoin_task(recv, client)).await;
                    }
                }
            }
        }
    }

    pub async fn init(kind: ShipKind, external_ip: Option<[u8; 4]>) -> anyhow::Result<Self> {
        let client = crate::net::Client::init(kind, external_ip).await;

        let client = Arc::new(tokio::sync::Mutex::new(client));
        let disconnect_handle = {
            info!("Registering for network...");
            let disconnect_handle = client.lock().await.register().await?;
            info!("Registered :)");
            disconnect_handle
        };

        // handle reconnecting to the network
        let recon_client = Arc::clone(&client);
        tokio::spawn(async move {
            Self::spawn_recursive_rejoin_task(disconnect_handle, recon_client).await;
        });

        Ok(Self { client })
    }
}
