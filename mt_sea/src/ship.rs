use std::sync::Arc;

use crate::net::NetArray;
use anyhow::anyhow;
use log::{debug, error, info};
use rkyv::{
    Archive, Deserialize,
    api::high::{HighValidator, from_bytes},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    to_bytes,
    util::AlignedVec,
};

use crate::{Sendable, ShipKind, VariableType, client::Client, net::PacketKind};

/// Copy bytes into an aligned buffer for rkyv deserialization
fn align_bytes(bytes: &[u8]) -> AlignedVec {
    let mut aligned = AlignedVec::with_capacity(bytes.len());
    aligned.extend_from_slice(bytes);
    aligned
}

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
            let data = to_bytes::<rkyv::rancor::Error>(data).expect("Could not serialize data");

            // Get the ship name from the target
            let target_ship_name = match &target.kind {
                ShipKind::Rat(name) => name.clone(),
                ShipKind::Wind(name) => name.clone(),
            };

            client
                .send_raw_to_other_client(id, data, variable_type, variable_name, &target_ship_name)
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

        // Loop until we get data for our id
        // We must check the buffer on EVERY iteration because notifications
        // might have been sent before we started waiting on the channel
        loop {
            // Check if data is already in buffer
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                // Data found - deserialize and return
                let mut out_buf = Vec::with_capacity(data_vec.len());
                for (raw, _, _) in data_vec {
                    let aligned = align_bytes(&raw);
                    let mat = from_bytes::<T, rkyv::rancor::Error>(&aligned)
                        .expect("Could not decode data to T");
                    out_buf.push(mat);
                }
                return Ok(out_buf);
            }

            // No data yet - wait for notification
            // Use recv() which blocks until a message arrives
            // If we miss a notification, the next iteration will check the buffer again
            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        // Our data might be ready, loop back to check buffer
                        continue;
                    }
                    // Not our id, keep waiting
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    // We missed some messages - that's fine, just check the buffer
                    debug!("Catch receiver lagged by {} messages, checking buffer", n);
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(anyhow!("Update channel closed while waiting for data"));
                }
            }
        }
    }

    async fn catch_dyn(&self, id: u32) -> anyhow::Result<Vec<(String, VariableType, String)>> {
        fn to_dyn_str(var_type: VariableType, buf: Vec<u8>) -> anyhow::Result<String> {
            let aligned = align_bytes(&buf);
            Ok(match var_type {
                VariableType::StaticOnly => {
                    return Err(anyhow!(
                        "Received Variable without dynamic type info, could not decode."
                    ));
                }
                VariableType::U8 => {
                    let deserialized = from_bytes::<NetArray<u8>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<u8> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::I32 => {
                    let deserialized = from_bytes::<NetArray<i32>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<i32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F32 => {
                    let deserialized = from_bytes::<NetArray<f32>, rkyv::rancor::Error>(&aligned)?;
                    let mat: nalgebra::DMatrix<f32> = deserialized.into();
                    format!("{:?}", mat)
                }
                VariableType::F64 => {
                    let deserialized = from_bytes::<NetArray<f64>, rkyv::rancor::Error>(&aligned)?;
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

        // Loop until we get data for our id
        // We must check the buffer on EVERY iteration because notifications
        // might have been sent before we started waiting on the channel
        loop {
            // Check if data is already in buffer
            let data_opt = {
                let mut buf_lock = buf.write().unwrap();
                buf_lock.remove(&id)
            };

            if let Some(data_vec) = data_opt {
                // Data found - convert and return
                let mut out_buf = Vec::with_capacity(data_vec.len());
                for (raw, var_type, var_name) in data_vec {
                    out_buf.push((to_dyn_str(var_type, raw)?, var_type, var_name));
                }
                return Ok(out_buf);
            }

            // No data yet - wait for notification
            match update_chan.recv().await {
                Ok(update_id) => {
                    if update_id == id {
                        // Our data might be ready, loop back to check buffer
                        continue;
                    }
                    // Not our id, keep waiting
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    // We missed some messages - that's fine, just check the buffer
                    debug!(
                        "Catch_dyn receiver lagged by {} messages, checking buffer",
                        n
                    );
                    continue;
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(anyhow!("Update channel closed while waiting for data"));
                }
            }
        }
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

            // Subscribe BEFORE sending request to avoid race condition
            let mut sub = client
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .map(|sub| sub.subscribe())
                .ok_or(anyhow!(
                    "Sender to Coordinator is available but Receiver is not."
                ))?;

            // Release client lock before sending to avoid deadlock
            drop(client);

            // Send the request
            sender.send(action_request).await?;

            // Wait for the response matching our variable
            loop {
                match sub.recv().await {
                    Ok((packet, _)) => {
                        if let PacketKind::RatAction {
                            variable,
                            action,
                            lock_until_ack,
                        } = packet.data
                        {
                            // Only accept responses for the variable we requested
                            if variable == variable_name {
                                return Ok((action, lock_until_ack));
                            }
                            // Wrong variable, keep waiting
                        }
                        // Not a RatAction, keep waiting
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!(
                            "ask_for_action receiver lagged by {} messages, continuing",
                            n
                        );
                        continue;
                    }
                    Err(e) => {
                        return Err(anyhow!(
                            "Could not receive answer for variable question from coordinator: {e}"
                        ));
                    }
                }
            }
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
        // Get the wind receiver and coordinator sender from client
        let (wind_receiver, coord_send) = {
            let client = self.client.lock().await;
            let receiver = std::sync::Arc::clone(&client.wind_receiver);
            let sender = client.coordinator_send.read().unwrap().clone();
            (receiver, sender)
        };

        let sender = coord_send.ok_or(anyhow!("Coordinator send not available"))?;

        // Lock the wind receiver and wait for the next packet
        let mut receiver = wind_receiver.lock().await;
        match receiver.recv().await {
            Some(packet) => {
                if let PacketKind::Wind(bwd) = packet.data {
                    // Send ack to coordinator
                    sender
                        .send(crate::net::Packet {
                            header: crate::net::Header::default(),
                            data: PacketKind::Acknowledge,
                        })
                        .await
                        .map_err(|e| anyhow!("Failed to send wind ack: {}", e))?;
                    return Ok(bwd.into_iter().map(|wa| wa.data).collect::<Vec<_>>());
                } else {
                    return Err(anyhow!("Expected Wind packet but got something else"));
                }
            }
            None => {
                return Err(anyhow!("Wind channel closed"));
            }
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

    pub async fn init(kind: ShipKind, rm_rules_on_disconnect: bool) -> anyhow::Result<Self> {
        let client = Client::init(kind.clone(), rm_rules_on_disconnect).await;

        let client = Arc::new(tokio::sync::Mutex::new(client));
        let _disconnect_handle = {
            info!("{:?} Registering for network...", &kind);
            let disconnect_handle = client.lock().await.register().await?;
            info!("{:?} Registered.", &kind);
            disconnect_handle
        };

        Ok(Self { client })
    }
}
