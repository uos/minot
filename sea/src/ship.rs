use std::{net::IpAddr, str::FromStr, sync::Arc};

use anyhow::anyhow;
use log::{debug, error, info};

use crate::{
    ShipKind, VariableType,
    client::Client,
    net::{PacketKind, SeaSendableBuffer},
};

pub struct NetworkShipImpl {
    pub client: Arc<tokio::sync::Mutex<Client>>,
}

#[async_trait::async_trait]
impl crate::Cannon for NetworkShipImpl {
    async fn shoot(
        &self,
        targets: &Vec<crate::NetworkShipAddress>,
        data: impl crate::net::SeaSendableBuffer,
        variable_type: VariableType,
        variable_name: &str,
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
                .send_raw_to_other_client(ip_addr, target.port, data, variable_type, &variable_name)
                .await?;
        }
        Ok(())
    }

    /// Catch the dumped data from the source.
    async fn catch<T: SeaSendableBuffer>(
        &self,
        target: &crate::NetworkShipAddress, // TODO rm target, since port is fixed?
    ) -> anyhow::Result<T> {
        let client = self.client.lock().await;
        let data = client.recv_raw_from_other_client(Some(target)).await?;
        let mat = T::set_from_packet(data.0).expect("Could not decode data to Matrix");
        Ok(mat)
    }
    async fn catch_dyn(
        &self,
        target: &crate::NetworkShipAddress,
    ) -> anyhow::Result<(String, VariableType, String)> {
        let client = self.client.lock().await;
        let (raw_data, var_type, var_name) =
            client.recv_raw_from_other_client(Some(target)).await?;
        let strrep = match var_type {
            VariableType::StaticOnly => {
                return Err(anyhow!(
                    "Received Variable without dynamic type info, could not decode."
                ));
            }
            VariableType::U8 => {
                let mat = nalgebra::DMatrix::<u8>::set_from_packet(raw_data)?;
                format!("{:?}", mat)
            }
            VariableType::I32 => {
                let mat = nalgebra::DMatrix::<i32>::set_from_packet(raw_data)?;
                format!("{:?}", mat)
            }
            VariableType::F32 => {
                let mat = nalgebra::DMatrix::<f32>::set_from_packet(raw_data)?;
                format!("{:?}", mat)
            }
            VariableType::F64 => {
                let mat = nalgebra::DMatrix::<f64>::set_from_packet(raw_data)?;
                format!("{:?}", mat)
            }
        };

        Ok((strrep, var_type, var_name))
    }
}

#[async_trait::async_trait]
impl crate::Ship for NetworkShipImpl {
    async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<(crate::Action, bool)> {
        let client = self.client.lock().await;
        let coord_send = client.coordinator_send.read().unwrap().clone();
        if let Some(sender) = coord_send {
            let action_request = crate::net::Packet {
                header: crate::net::Header::default(),
                data: PacketKind::VariableTaskRequest(variable_name.to_string()),
            };

            let mut sub = client
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .map(|sub| sub.subscribe())
                .ok_or(anyhow!(
                    "Sender to Coordinator is available but Receiver is not."
                ))?;

            let (tx, mut rx) = tokio::sync::mpsc::channel(1);
            tokio::spawn(async move {
                loop {
                    match sub.recv().await {
                        Ok((packet, _)) => {
                            if let PacketKind::RatAction {
                                action,
                                lock_until_ack,
                            } = packet.data
                            {
                                tx.send(Ok((action, lock_until_ack))).await.unwrap();
                                return;
                            }
                        }
                        Err(e) => {
                            tx.send(Err(anyhow!(
                                        "Could not receive answer for variable question from coordinator: {e}"
                                    )))
                                        .await
                                        .unwrap();
                        }
                    }
                }
            });
            sender.send(action_request).await?;
            return rx.recv().await.unwrap();
        } else {
            drop(client);
            tokio::task::yield_now().await;
            return self.ask_for_action(&variable_name).await;
        }
    }

    fn get_cannon(&self) -> &impl crate::Cannon {
        self
    }

    // async fn wait_for_action(&self) -> anyhow::Result<crate::Action> {
    //     let client = self.client.lock().await;
    //     let coord_receive = client.coordinator_receive.read().unwrap().clone();
    //     if let Some(receiver) = coord_receive {
    //         let mut sub = receiver.subscribe();
    //         debug!("subbed");
    //         while let Ok((packet, _)) = sub.recv().await {
    //             debug!("received");
    //             if let PacketKind::RatAction(action) = packet.data {
    //                 return Ok(action);
    //             }
    //         }
    //         return Err(anyhow!(
    //             "Could not receive action while Channel was subscribed."
    //         ));
    //     } else {
    //         debug!("waiting to be available..");
    //         tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    //         return self.wait_for_action().await;
    //     }
    // }

    async fn wait_for_wind(&self) -> anyhow::Result<crate::WindData> {
        let client = self.client.lock().await;
        let coord_receive = client.coordinator_receive.read().unwrap().clone();
        if let Some(receiver) = coord_receive {
            let mut sub = receiver.subscribe();
            while let Ok((packet, _)) = sub.recv().await {
                if let PacketKind::Wind(data) = packet.data {
                    return Ok(data);
                }
            }
            return Err(anyhow!(
                "Could not receive wind while Channel was subscribed."
            ));
        } else {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            return self.wait_for_wind().await;
        }
    }
}

impl NetworkShipImpl {
    #[allow(dead_code)]
    async fn spawn_recursive_rejoin_task(
        disconnect_handle: tokio::sync::oneshot::Receiver<()>,
        client: Arc<tokio::sync::Mutex<Client>>,
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
        let client = Client::init(kind, external_ip).await;

        let client = Arc::new(tokio::sync::Mutex::new(client));
        let _disconnect_handle = {
            info!("Registering for network...");
            let disconnect_handle = client.lock().await.register().await?;
            info!("Registered :)");
            disconnect_handle
        };

        // handle reconnecting to the network
        // TODO handle reconnect, can not rejoin task because
        // let recon_client = Arc::clone(&client);
        // tokio::spawn(async move {
        // Self::spawn_recursive_rejoin_task(disconnect_handle, recon_client).await;
        // });

        Ok(Self { client })
    }

    pub async fn next_catch(&self) -> anyhow::Result<crate::NetworkShipAddress> {
        let mut sub = {
            let client = self.client.lock().await;
            let sub = client
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .map(|sub| sub.subscribe())
                .ok_or(anyhow!(
                    "Sender to Coordinator is available but Receiver is not."
                ))?;
            sub
        };

        debug!("waiting");
        loop {
            let (packet, _) = sub.recv().await.map_err(|e| {
                anyhow!("Could not receive answer for variable question from coordinator: {e}")
            })?;
            debug!("received something");
            match packet.data {
                PacketKind::RatAction {
                    action,
                    lock_until_ack: _,
                } => {
                    if let crate::Action::Catch { source } = action {
                        debug!("received rat action");
                        return Ok(source);
                    }
                }
                _ => (),
            }
        }
    }
}
