use anyhow::anyhow;
use log::{error, info};
use rkyv::{
    Archive, Deserialize, Serialize,
    api::high::{HighSerializer, HighValidator},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    ser::allocator::ArenaHandle,
    util::AlignedVec,
};
use std::sync::Arc;

use sea::{ship::NetworkShipImpl, *};

pub struct Node {
    name: String,
    ship: Arc<NetworkShipImpl>,
}

impl Node {
    pub async fn create(name: &str) -> anyhow::Result<Self> {
        let ship = sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_string()), None).await?;

        Ok(Self {
            name: name.to_string(),
            ship: Arc::new(ship),
        })
    }

    pub async fn publish(
        &self,
        topic: &str,
        data: &(
             impl for<'a> Serialize<HighSerializer<AlignedVec, ArenaHandle<'a>, rkyv::rancor::Error>>
             + Send
             + Sync
         ),
    ) -> anyhow::Result<()> {
        match self.ship.ask_for_action(topic).await {
            Ok((sea::Action::Sail, _)) => {
                info!("Doing nothing but expected a shoot command {} ", topic);
                return Ok(());
            }
            Ok((sea::Action::Shoot { target }, _)) => {
                info!("RatNode {} shoots {} at {:?}", self.name, topic, target);

                self.ship
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

    pub async fn subscribe<T>(
        &self,
        topic: &str,
        queue_size: usize,
    ) -> anyhow::Result<tokio::sync::mpsc::Receiver<T>>
    where
        T: Archive,
        T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
        T: 'static + Send,
    {
        let rat_ship = Arc::clone(&self.ship);
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
        return Ok(rx);
    }
}

// pub struct NodeStatic {
//     name: String,
//     ship: Option<Arc<NetworkShipImpl>>,
//     task: std::thread::JoinHandle<()>,
// }

// impl NodeStatic {
//     pub fn create(name: &str) -> anyhow::Result<Self> {
//         let (fail_tx, fail_rx) = std::sync::mpsc::channel();
//         let (res_tx, res_rx) = std::sync::mpsc::channel();
//         let handle = std::thread::spawn(move || {
//             let rt = tokio::runtime::Builder::new_current_thread()
//                 .worker_threads(1)
//                 .build();
//             let rt = match rt {
//                 Ok(rt) => rt,
//                 Err(e) => {
//                     fail_tx.send(Some(Err(e.into()))).unwrap();
//                     return;
//                 }
//             };

//             rt.spawn(async move {
//                 let node = Node::create(name).await;
//                 let node = match node {
//                     Ok(node) => node,
//                     Err(e) => {
//                         fail_tx.send(Some(Err(e.into()))).unwrap();
//                         return;
//                     }
//                 };

//                 fail_tx.send(None).unwrap();

//                 let pubber_node = Arc::new(&node);
//                 let subber_node = Arc::clone(&pubber_node);

//                 let (subber_tx, subber_rx) = tokio::sync::mpsc::channel(100);

//                 // subscriber
//                 tokio::spawn(async move {
//                     while let Some((topic, queue_size)) = subber_rx.recv().await {
//                         let res = subber_node.subscribe(topic, queue_size).await;
//                         match res {
//                             Ok(receiver) => {
//                                 tokio::spawn(async move {
//                                     while let Some(r) = receiver.recv().await {
//                                         // we can not know statically which type to deserialize, so there is no way to call this dynamically in this at this stage. the function would be generic to just one type
//                                     }
//                                 })
//                             }
//                             Err(_) => todo!(),
//                         }
//                     }
//                 });
//             });

//             loop {
//                 let res = fail_rx.recv().unwrap();
//             }

//             // TODO start listening on channel for publish and subscribe tasks, translate to tokio channel and forward to tasks
//         });

//         loop {
//             let failed = fail_rx.recv().unwrap();
//             if let Some(e) = failed {
//                 return e;
//             }
//             break;
//         }

//         Ok(Self {
//             name: name.to_string(),
//             ship: Some(Arc::new(ship)),
//         })
//     }

//     pub async fn publish(
//         &self,
//         topic: &str,
//         data: &(
//              impl for<'a> Serialize<HighSerializer<AlignedVec, ArenaHandle<'a>, rkyv::rancor::Error>>
//              + Send
//              + Sync
//          ),
//     ) -> anyhow::Result<()> {
//         if let Some(rat_ship) = self.ship.as_ref() {
//             match rat_ship.ask_for_action(topic).await {
//                 Ok((sea::Action::Sail, _)) => {
//                     info!("Doing nothing but expected a shoot command {} ", topic);
//                     return Ok(());
//                 }
//                 Ok((sea::Action::Shoot { target }, _)) => {
//                     info!("RatNode {} shoots {} at {:?}", self.name, topic, target);

//                     rat_ship
//                         .get_cannon()
//                         .shoot(&target, data, VariableType::StaticOnly, topic)
//                         .await?;

//                     info!(
//                         "Rat {} finished shooting {} at {:?}",
//                         self.name, topic, target
//                     );

//                     return Ok(());
//                 }
//                 Ok((sea::Action::Catch { source: _ }, _)) => {
//                     return Err(anyhow!(
//                         "Received Catch but we are in a publisher for {} ",
//                         topic
//                     ));
//                 }
//                 Err(e) => {
//                     return Err(e);
//                 }
//             };
//         }
//         Err(anyhow!("ship not initialized"))
//     }

//     pub async fn subscribe<T>(
//         &self,
//         topic: &str,
//         queue_size: usize,
//     ) -> anyhow::Result<tokio::sync::mpsc::Receiver<T>>
//     where
//         T: Archive,
//         T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
//             + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
//         T: 'static + Send,
//     {
//         if let Some(rat_ship) = self.ship.as_ref() {
//             let rat_ship = Arc::clone(&rat_ship);
//             let name = self.name.clone();
//             let topic = topic.to_owned();
//             let (tx, rx) = tokio::sync::mpsc::channel(queue_size);
//             tokio::spawn(async move {
//                 loop {
//                     match rat_ship.ask_for_action(&topic).await {
//                         Ok((sea::Action::Sail, _)) => {
//                             info!("Doing nothing but expected a catch command {} ", topic);
//                             continue;
//                         }
//                         Ok((sea::Action::Shoot { target: _ }, _)) => {
//                             error!("Received Shoot but we are in a subscriber for {} ", topic);
//                             continue;
//                         }
//                         Ok((sea::Action::Catch { source }, _)) => {
//                             let recv_data = rat_ship.get_cannon().catch::<T>(&source).await;
//                             let recv_data = match recv_data {
//                                 Ok(recv_data) => recv_data,
//                                 Err(e) => {
//                                     error!("Error catching: {e}");
//                                     return;
//                                 }
//                             };
//                             info!(
//                                 "RatNode {} finished catching {} from {:?}",
//                                 name, topic, source
//                             );
//                             let sent = tx.send(recv_data).await;
//                             match sent {
//                                 Ok(_) => {}
//                                 Err(e) => {
//                                     error!(
//                                         "Error sending received buffer out through channel: {e}"
//                                     );
//                                     return;
//                                 }
//                             }
//                         }
//                         Err(e) => {
//                             error!("Error asking for action: {e}");
//                             return; // stopping spawned loop will drop the tx, therefore closing it
//                         }
//                     };
//                 }
//             });
//             return Ok(rx);
//         }
//         Err(anyhow!("Rat not initialized"))
//     }
// }
