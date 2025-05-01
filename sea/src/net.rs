use std::{
    collections::{HashMap, HashSet},
    net::{Ipv4Addr, SocketAddr, TcpStream},
    str::FromStr,
};

use anyhow::anyhow;
use log::{debug, error, info, warn};
use nalgebra::DMatrix;

use serde::{Deserialize, Serialize};

use crate::{Action, ShipKind, ShipName, VariableHuman, WindData, client::Client};

pub const PROTO_IDENTIFIER: u8 = 69;
pub const CONTROLLER_CLIENT_ID: ShipName = 0;
pub const CLIENT_REGISTER_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(150);
pub const CLIENT_LISTEN_PORT: u16 = 6969;
pub const CLIENT_REJOIN_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);
pub const CLIENT_HEARTBEAT_TCP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
pub const CLIENT_HEARTBEAT_TCP_INTERVAL: std::time::Duration =
    std::time::Duration::from_millis(200);
pub const SERVER_DROP_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(200);
pub const CLIENT_TO_CLIENT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(60); // TODO or never?
pub const CLIENT_TO_CLIENT_INIT_RETRY_TIMEOUT: std::time::Duration =
    std::time::Duration::from_millis(50);

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum PacketKind {
    Acknowledge,
    RequestVarSend(String),
    JoinRequest(u16, u16, ShipKind),
    Welcome(crate::NetworkShipAddress), // the id of the rat so the coordinator can differentiate them and the tcp port for 1:1 and heartbeat
    Heartbeat,
    Disconnect,
    RuleAppend {
        variable: String,
        commands: Vec<VariableHuman>,
    },
    RulesClear,
    LockNext, // TODO lock next variable so after a client does catch/shoot, it blocks until receiving an ack. The initial ack is sent from lh to the coordinator, that then sends ack to all connected clients
    Unlock,
    RawDataf64(nalgebra::DMatrix<f64>),
    RawDataf32(nalgebra::DMatrix<f32>),
    RawDatai32(nalgebra::DMatrix<i32>),
    RawDatau8(nalgebra::DMatrix<u8>),
    VariableTaskRequest(String),
    RatAction {
        action: Action,
        lock_until_ack: bool,
    },
    Wind {
        data: WindData,
        at_var: Option<String>,
    },
}

pub trait SeaSendableScalar:
    nalgebra::Scalar + serde::Serialize + for<'a> serde::Deserialize<'a>
{
}
impl SeaSendableScalar for f64 {}
impl SeaSendableScalar for f32 {}
impl SeaSendableScalar for u8 {}
impl SeaSendableScalar for i32 {}

pub trait SeaSendableBuffer: Send + Clone {
    fn to_packet(self) -> Vec<u8>;
    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self>;
}

impl SeaSendableBuffer for () {
    fn to_packet(self) -> Vec<u8> {
        unimplemented!("Should only be a shadow for async_trait crate.")
    }

    fn set_from_packet(_: Vec<u8>) -> anyhow::Result<Self> {
        unimplemented!("Should only be a shadow for async_trait crate.")
    }
}

impl SeaSendableBuffer for DMatrix<u8> {
    fn to_packet(self) -> Vec<u8> {
        bincode::serialize(&PacketKind::RawDatau8(self)).expect("data not serializable")
    }

    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self> {
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDatau8(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }
}

impl SeaSendableBuffer for DMatrix<f64> {
    fn to_packet(self) -> Vec<u8> {
        bincode::serialize(&PacketKind::RawDataf64(self)).expect("data not serializable")
    }

    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self> {
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDataf64(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }
}
impl SeaSendableBuffer for DMatrix<f32> {
    fn to_packet(self) -> Vec<u8> {
        bincode::serialize(&PacketKind::RawDataf32(self)).expect("data not serializable")
    }

    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self> {
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDataf32(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }
}
impl SeaSendableBuffer for DMatrix<i32> {
    fn to_packet(self) -> Vec<u8> {
        bincode::serialize(&PacketKind::RawDatai32(self)).expect("data not serializable")
    }

    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self> {
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDatai32(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }
}

// With a join request, the client sends a joinrequest with udp to all
// available broadcast addresses. The UDP port is not important here.
// The target port is the fixed udp port of the server.
// Since we only have one fixed port on a device, the coordinator,
// the clients need to use dynamic ports everywhere else.
//
// The coordinator listens to these requests on the fixed port.
// The join request includes the tcp listener port of the client.
// The coordinator must save this port. It now always uses that when communicating with it and it also sends to it other clients if they need
// to have a connection.

#[derive(Serialize, Deserialize, Copy, Clone, Debug, Default)]
pub struct Header {
    pub source: ShipName,
    pub target: ShipName,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Packet {
    pub header: Header,
    pub data: PacketKind,
}

#[derive(Clone, Debug)]
pub struct ShipHandle {
    pub name: ShipKind,
    pub addr_from_coord: crate::NetworkShipAddress,
    pub ship: ShipName,
    // Send here to disconnect the tcp listener
    pub disconnect: tokio::sync::broadcast::Sender<bool>,
    // get requests from the client
    pub recv: tokio::sync::broadcast::Sender<(Packet, Option<SocketAddr>)>,
    // send to this client
    pub send: tokio::sync::mpsc::Sender<Packet>,
    pub other_client_port: u16,
}

#[derive(Debug)]
pub struct Sea {
    pub network_clients_chan: tokio::sync::broadcast::Sender<ShipHandle>,
    dissolve_network: tokio::sync::mpsc::Sender<tokio::sync::mpsc::Sender<()>>,
}

/// Server handling the Sea network
impl Sea {
    pub async fn init(external_ip: Option<[u8; 4]>) -> Self {
        let (rejoin_req_tx, mut rejoin_req_rx) = tokio::sync::mpsc::channel::<(String, Packet)>(10);

        let (clients_tx, mut clients_rx) = tokio::sync::broadcast::channel::<ShipHandle>(10);

        let (dissolve_network_tx, mut dissolve_network_rx) =
            tokio::sync::mpsc::channel::<tokio::sync::mpsc::Sender<()>>(10);

        // task to disconnect all clients i.e. dissolve
        tokio::spawn(async move {
            let mut clients: Vec<tokio::sync::broadcast::Sender<bool>> = Vec::new();
            loop {
                tokio::select! {
                    answer = dissolve_network_rx.recv() => {
                        match answer {
                            None => {
                                // channel closed
                                return;
                            }
                            Some(answer) => {
                                for c in clients.iter() {
                                    c.send(true).unwrap();
                                }
                                // notify that we are finished
                                answer.send(()).await.unwrap();
                                return;
                            }
                        }
                    }
                    newclient = clients_rx.recv() => {
                        match newclient {
                            Err(e) => {
                                error!("Error receiving new client in dissolve handler: {e}");
                            }
                            Ok(client) => {
                                clients.push(client.disconnect);
                            }
                        }
                    }
                }
            }
        });

        // task to handle join requests on udp socket
        tokio::spawn(async move {
            let udp_listener = Client::get_udp_socket(external_ip, Some(CLIENT_LISTEN_PORT)).await;
            let rejoin_request = Packet {
                header: Header {
                    source: ShipName::MAX,
                    target: CONTROLLER_CLIENT_ID,
                },
                // names are padded with maximal length 64 chars
                data: PacketKind::JoinRequest(
                    0,
                    0,
                    Sea::pad_ship_kind_name(&ShipKind::Rat("".to_string())),
                ),
            };
            let bytes_rejoin_request =
                bincode::serialize(&rejoin_request).expect("could not serialize rejoin request");
            let expected_n_bytes_for_rejoin_request = bytes_rejoin_request.len();
            let mut new_clients_without_response = HashMap::<String, (usize, Vec<u8>)>::new();

            info!("Listening {:?}", udp_listener.local_addr().unwrap());
            debug!(
                "Expecting {} bytes for JoinRequest including PROTO_IDENTIFIER",
                expected_n_bytes_for_rejoin_request + 1
            );

            loop {
                let mut buf = [0; 256]; // JoinRequest normally 115 bytes
                let (n, addr) = udp_listener.recv_from(&mut buf).await.unwrap();
                let id = format!("{}:{}", addr.ip(), addr.port());
                debug!("Receiving {} bytes from {} via UDP", n, id);

                match new_clients_without_response.get_mut(&id) {
                    Some((kum, buffer)) => {
                        *kum += n;
                        buffer.extend_from_slice(&buf[..n]);
                    }
                    None => {
                        let mut buffer = Vec::with_capacity(1024);
                        if buf[0] != PROTO_IDENTIFIER {
                            continue; // not meant for us
                        } else {
                            buffer.extend_from_slice(&buf[1..n]);
                        }
                        new_clients_without_response.insert(id, (buffer.len(), buffer));
                    }
                }

                let mut to_delete = Vec::<String>::new();
                for (id, (kum, buffer)) in new_clients_without_response.iter_mut() {
                    if *kum != expected_n_bytes_for_rejoin_request {
                        continue;
                    }

                    let packet: Packet = match bincode::deserialize(&buffer) {
                        Err(e) => {
                            error!("Received package is broken: {e}");
                            continue;
                        }
                        Ok(packet) => packet,
                    };

                    match rejoin_req_tx.send((id.clone(), packet)).await {
                        Err(e) => {
                            error!("Could not send rejoin request to internal channel: {e}");
                        }
                        Ok(_) => {
                            to_delete.push(id.clone());
                        }
                    };
                }

                for id in to_delete {
                    new_clients_without_response.remove(&id);
                }

                tokio::task::yield_now().await; // needed to yield to receiver, else the thread is sometimes blocking
            }
        });

        // task to wait for each new join request
        let clients_tx_inner = clients_tx.clone();
        tokio::spawn(async move {
            let rat_lock = std::sync::Arc::new(std::sync::Mutex::new(HashSet::new()));
            // let rat_lock_unlock = Arc::clone(&rat_lock); // currently we only allow unique names, so no unlock needed.
            loop {
                let receive = rejoin_req_rx.recv().await;
                if let Some((addr, packet)) = receive {
                    match packet.data {
                        PacketKind::JoinRequest(client_tcp_port, other_client_port, ship_kind) => {
                            let ship_kind = Sea::unpad_ship_kind_name(&ship_kind);
                            debug!("Received RejoinRequest: {:?} from {:?}", ship_kind, addr);
                            {
                                let mut lock = rat_lock.lock().unwrap();
                                if let Some(_) = lock.get(&ship_kind) {
                                    debug!(
                                        "requested client already exists or is in the progress of joining the network"
                                    );
                                    continue;
                                }
                                lock.insert(ship_kind.clone());
                            }
                            let generated_id = rand::random::<ShipName>().abs();
                            let (disconnect_tx, _disconnect_rx) =
                                tokio::sync::broadcast::channel::<bool>(1);

                            // task to handle tcp connection to this client
                            let curr_client_create_sender = clients_tx_inner.clone();
                            tokio::spawn(async move {
                                let ip = addr.split(':').next().unwrap();
                                let client_stream = tokio::net::TcpStream::connect(format!(
                                    "{}:{}",
                                    ip, client_tcp_port
                                ))
                                .await
                                .expect("could not connect to client");

                                let socket =
                                    socket2::Socket::from(client_stream.into_std().unwrap());
                                socket.set_keepalive(true).unwrap();

                                // socket
                                //     .set_tcp_keepalive(
                                //         &socket2::TcpKeepalive::new()
                                //             .with_time(CLIENT_HEARTBEAT_TCP_TIMEOUT)
                                //             .with_interval(CLIENT_HEARTBEAT_TCP_INTERVAL),
                                //     )
                                //     .unwrap();
                                socket
                                    .set_linger(Some(std::time::Duration::from_secs(30)))
                                    .unwrap();
                                let stream: TcpStream = socket.into();
                                let client_stream =
                                    tokio::net::TcpStream::from_std(stream).unwrap();

                                let (rh, wh) = client_stream.into_split();

                                let (tx, _) = tokio::sync::broadcast::channel::<(
                                    Packet,
                                    Option<SocketAddr>,
                                )>(10);
                                let tx_out = tx.clone();

                                let (client_sender_tx, client_sender_rx) =
                                    tokio::sync::mpsc::channel::<Packet>(10);

                                // reading stream from client
                                let current_ship = ship_kind.clone();
                                tokio::spawn(async move {
                                    Client::receive_from_socket(rh, tx, None).await;
                                    warn!("Client {:?} disconnected.", current_ship);
                                    // TODO when here, the client is disconnected, so stop all processing (block or send wait signal) until the clients are back
                                });

                                // writing stream to client
                                tokio::spawn(async move {
                                    Client::send_to_socket(client_sender_rx, wh).await;
                                });

                                let ip_parsed = Ipv4Addr::from_str(ip).expect("Strange ip format");

                                let client_addr = crate::NetworkShipAddress {
                                    ip: ip_parsed.octets(),
                                    port: client_tcp_port,
                                    ship: generated_id,
                                    kind: ship_kind.clone(),
                                };

                                let welcome_packet = Packet {
                                    header: Header {
                                        source: CONTROLLER_CLIENT_ID,
                                        target: generated_id,
                                    },
                                    data: PacketKind::Welcome(client_addr.clone()),
                                };

                                match client_sender_tx.send(welcome_packet).await {
                                    Ok(_) => {}
                                    Err(e) => {
                                        error!("Could not send welcome packet to channel: {e}");
                                    }
                                }

                                let ship_handle = ShipHandle {
                                    ship: generated_id,
                                    disconnect: disconnect_tx,
                                    recv: tx_out,
                                    send: client_sender_tx,
                                    name: ship_kind,
                                    addr_from_coord: client_addr,
                                    other_client_port,
                                };
                                curr_client_create_sender.send(ship_handle).unwrap();
                                debug!("ShipHandle created and sent");
                            });
                        }
                        _ => {
                            warn!("Received unexpected packet: {packet:?}");
                        }
                    }
                } else {
                    error!("Channel closed, could not receive rejoin requests in channel.");
                }
            }
        });

        Self {
            network_clients_chan: clients_tx,
            dissolve_network: dissolve_network_tx,
        }
    }

    pub fn pad_string(input: &str) -> String {
        if input.len() >= 64 {
            return input.to_string(); // Return the string if it's already 64 or longer
        }
        let padding_count = 64 - input.len();
        let padding = "#".repeat(padding_count);
        format!("{}{}", input, padding)
    }

    pub fn reverse_padding(input: &str) -> String {
        let trimmed: &str = input.trim_end_matches('#');
        trimmed.to_string()
    }

    pub fn pad_ship_kind_name(kind: &ShipKind) -> ShipKind {
        match kind {
            ShipKind::Rat(name) => ShipKind::Rat(Self::pad_string(name)),
            ShipKind::Wind(name) => ShipKind::Wind(Self::pad_string(name)),
        }
    }

    pub fn unpad_ship_kind_name(kind: &ShipKind) -> ShipKind {
        match kind {
            ShipKind::Rat(name) => ShipKind::Rat(Self::reverse_padding(name)),
            ShipKind::Wind(name) => ShipKind::Wind(Self::reverse_padding(name)),
        }
    }

    pub async fn cleanup(&mut self) {
        let (answer_tx, mut answer_rx) = tokio::sync::mpsc::channel(1);
        match self.dissolve_network.send(answer_tx).await {
            Err(e) => {
                error!("Error while droppping network: {e}");
            }
            Ok(_) => {}
        }

        let answer_timeout = tokio::time::timeout(SERVER_DROP_TIMEOUT, answer_rx.recv());
        // tokio::task::yield_now().await;
        match answer_timeout.await {
            Err(e) => {
                error!("Dropping network timeout, discarding waiting for completion: {e}");
            }
            Ok(None) => {
                warn!("Sender already closed in dissolving answer");
            }
            _ => {}
        }
    }
}

// TODO block_in_place blocks the current thread. So when the dissolve_network receivers are running on the same thread, is this a deadlock since we are blocking the thread?
impl Drop for Sea {
    fn drop(&mut self) {
        tokio::task::block_in_place(move || {
            let rt = tokio::runtime::Handle::current();
            rt.block_on(self.cleanup());
        });
    }
}

// impl std::future::AsyncDrop for Sea {
//     type Dropper<'a> = impl std::future::Future<Output = ()>;

//     fn async_drop(self: std::pin::Pin<&mut Self>) -> Self::Dropper<'_> {
//         println!("calling async drop");
//         async move {
//             let (answer_tx, mut answer_rx) = tokio::sync::mpsc::channel(1);
//             match self.dissolve_network.send(answer_tx).await {
//                 Err(e) => {
//                     error!("Error while droppping network: {e}");
//                 }
//                 Ok(_) => {}
//             }

//             let answer_timeout = tokio::time::timeout(SERVER_DROP_TIMEOUT, answer_rx.recv());
//             match answer_timeout.await {
//                 Err(e) => {
//                     error!("Dropping network timeout, discarding waiting for completion: {e}");
//                 }
//                 Ok(None) => {
//                     error!("Sender already closed in dissolving answer");
//                 }
//                 _ => {}
//             }
//         }
//     }
// }
