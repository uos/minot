use std::{net::IpAddr, str::FromStr, sync::Arc};

use log::{error, info};
use nalgebra::SimdValue;

use crate::{
    net::{Client, SeaSendableBuffer},
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
    ) {
        let client = self.client.lock().await;
        for target in targets.iter() {
            let data = data.clone().to_packet();
            let ip_addr = &IpAddr::from_str(&format!(
                "{}.{}.{}.{}",
                target.ip[0], target.ip[1], target.ip[2], target.ip[3]
            ))
            .unwrap();
            let res = client
                .send_raw_to_other_client(ip_addr, target.port, data)
                .await;

            //             // client
            //     .send_raw_to_other_client(
            //         &IpAddr::from_str(&format!(
            //             "{}.{}.{}.{}",
            //             target.ip[0], target.ip[1], target.ip[2], target.ip[3]
            //         ))
            //         .unwrap(),
            //         target.port,
            //         data,
            //     )
            //     .await;
        }
    }

    /// Catch the dumped data from the source.
    async fn catch(&self, target: &crate::NetworkShipAddress) -> impl SeaSendableBuffer {
        todo!()
    }
}

#[async_trait::async_trait]
impl crate::Ship for NetworkShipImpl {
    async fn wait_for_action(&self, kind: crate::ShipKind) -> anyhow::Result<crate::Action> {
        todo!()
    }

    async fn ask_for_action(
        &self,
        kind: crate::ShipKind,
        variable_name: &str,
    ) -> anyhow::Result<crate::Action> {
        todo!()
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

impl Drop for NetworkShipImpl {
    fn drop(&mut self) {
        todo!("disconnect from network")
    }
}
