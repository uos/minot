use std::{net::IpAddr, str::FromStr, sync::Arc};

use anyhow::anyhow;
use log::{error, info};
use rkyv::{
    Archive, Deserialize,
    api::high::{HighValidator, from_bytes},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    to_bytes,
};

use crate::{
    Sendable, ShipKind, VariableType,
    client::Client,
    net::{NetArray, PacketKind},
};

#[derive(Debug)]
pub struct NetworkShipImpl {
    pub client: Arc<tokio::sync::Mutex<Client>>,
}

#[async_trait::async_trait]
impl crate::Cannon for NetworkShipImpl {
    async fn shoot<'b, T: Sendable>(
        &self,
        targets: &'b [crate::NetworkShipAddress],
        id: u32,
        data: &T,
        variable_type: VariableType,
        variable_name: &str,
    ) -> anyhow::Result<()> {
        let client = self.client.lock().await;
        for target in targets.iter() {
            let data = to_bytes::<rkyv::rancor::Error>(data).expect("Could not deserialize packet");
            let ip_addr = &IpAddr::from_str(&format!(
                "{}.{}.{}.{}",
                target.ip[0], target.ip[1], target.ip[2], target.ip[3]
            ))
            .unwrap();
            client
                .send_raw_to_other_client(
                    ip_addr,
                    id,
                    target.port,
                    data,
                    variable_type,
                    variable_name,
                )
                .await?;
        }
        Ok(())
    }

    /// Catch the dumped data from the source.
    async fn catch<T>(&self, id: u32) -> anyhow::Result<Vec<T>>
    where
        T: Send,
        T: Archive,
        T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    {
        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        let update_id = {
            let mut buf_lock = buf.write().unwrap();
            buf_lock.remove(&id)
        };

        let mut out_buf = Vec::new();
        match update_id {
            Some(data_vec) => {
                // missed update call, the data is already waiting for us. gather all.
                for (raw, _, _) in data_vec {
                    let mat = from_bytes::<T, rkyv::rancor::Error>(&raw)
                        .expect("Could not decode data to T");
                    out_buf.push(mat);
                }
            }
            None => loop {
                // wait for update call for our id
                if let Ok(update_id) = update_chan.recv().await {
                    if update_id == id {
                        let data_vec = {
                            let mut buf_lock = buf.write().unwrap();
                            match buf_lock.remove(&id) {
                                Some(v) => v,
                                None => continue,
                            }
                        };
                        for (raw, _, _) in data_vec {
                            let mat = from_bytes::<T, rkyv::rancor::Error>(&raw)
                                .expect("Could not decode data to T");
                            out_buf.push(mat);
                        }
                        break;
                    }
                }
            },
        };
        Ok(out_buf)
    }

    async fn catch_dyn(&self, id: u32) -> anyhow::Result<Vec<(String, VariableType, String)>> {
        fn to_dyn_str(var_type: VariableType, buf: Vec<u8>) -> anyhow::Result<String> {
            Ok(match var_type {
                VariableType::StaticOnly => {
                    return Err(anyhow!(
                        "Received Variable without dynamic type info, could not decode."
                    ));
                }
                VariableType::U8 => {
                    let deserialized = from_bytes::<NetArray<u8>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<u8> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::I32 => {
                    let deserialized = from_bytes::<NetArray<i32>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<i32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F32 => {
                    let deserialized = from_bytes::<NetArray<f32>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<f32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F64 => {
                    let deserialized = from_bytes::<NetArray<f64>, rkyv::rancor::Error>(&buf)?;
                    let mat: nalgebra::DMatrix<f64> = deserialized.into();
                    format!("{:?}", mat)
                }
            })
        }

        let (buf, mut update_chan) = {
            let client = self.client.lock().await;
            let buf = std::sync::Arc::clone(&client.raw_recv_buff);
            let update_chan = client.updated_raw_recv.subscribe();
            (buf, update_chan)
        };

        let update_id = {
            let mut buf_lock = buf.write().unwrap();
            buf_lock.remove(&id)
        };

        let mut out_buf = Vec::new();
        match update_id {
            Some(data_vec) => {
                // missed update call, the data is already waiting for us. gather all.
                for (raw, var_type, var_name) in data_vec {
                    out_buf.push((to_dyn_str(var_type, raw)?, var_type, var_name));
                }
            }
            None => loop {
                // wait for update call for our id
                if let Ok(update_id) = update_chan.recv().await {
                    if update_id == id {
                        let data_vec = {
                            let mut buf_lock = buf.write().unwrap();
                            match buf_lock.remove(&id) {
                                Some(v) => v,
                                None => continue,
                            }
                        };
                        for (raw, var_type, var_name) in data_vec {
                            out_buf.push((to_dyn_str(var_type, raw)?, var_type, var_name));
                        }
                        break;
                    }
                }
            },
        };

        Ok(out_buf)
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
            return self.ask_for_action(variable_name).await;
        }
    }

    fn get_cannon(&self) -> &impl crate::Cannon {
        self
    }

    async fn wait_for_wind(&self) -> anyhow::Result<Vec<crate::WindData>> {
        let client = self.client.lock().await;
        let coord_receive = { client.coordinator_receive.read().unwrap().clone() };
        if let Some(receiver) = coord_receive {
            let mut sub = receiver.subscribe();
            while let Ok((packet, _)) = sub.recv().await {
                if let PacketKind::Wind(bwd) = packet.data {
                    // send ack to coordinator
                    let sender = {
                        let coord_read = client.coordinator_send.read().unwrap();
                        coord_read.as_ref().unwrap().clone()
                    };
                    sender
                        .send(crate::net::Packet {
                            header: crate::net::Header::default(),
                            data: PacketKind::Acknowledge,
                        })
                        .await
                        .unwrap();
                    return Ok(bwd.into_iter().map(|wa| wa.data).collect::<Vec<_>>());
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

    pub async fn init(
        kind: ShipKind,
        external_ip: Option<[u8; 4]>,
        rm_rules_on_disconnect: bool,
    ) -> anyhow::Result<Self> {
        let client = Client::init(kind.clone(), external_ip, rm_rules_on_disconnect).await;

        let client = Arc::new(tokio::sync::Mutex::new(client));
        let _disconnect_handle = {
            info!("{:?} Registering for network...", &kind);
            let disconnect_handle = client.lock().await.register().await?;
            info!("{:?} Registered.", &kind);
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

    // pub async fn next_catch(&self) -> anyhow::Result<crate::NetworkShipAddress> {
    //     let mut sub = {
    //         let client = self.client.lock().await;
    //         let sub = client
    //             .coordinator_receive
    //             .read()
    //             .unwrap()
    //             .as_ref()
    //             .map(|sub| sub.subscribe())
    //             .ok_or(anyhow!(
    //                 "Sender to Coordinator is available but Receiver is not."
    //             ))?;
    //         sub
    //     };

    //     debug!("waiting");
    //     loop {
    //         let (packet, _) = sub.recv().await.map_err(|e| {
    //             anyhow!("Could not receive answer for variable question from coordinator: {e}")
    //         })?;
    //         debug!("received something");
    //         match packet.data {
    //             PacketKind::RatAction {
    //                 action,
    //                 lock_until_ack: _,
    //             } => {
    //                 if let crate::Action::Catch { source, id: _ } = action {
    //                     debug!("received rat action");
    //                     return Ok(source);
    //                 }
    //             }
    //             _ => (),
    //         }
    //     }
    // }
}
