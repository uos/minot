use std::{
    collections::HashMap,
    net::{IpAddr, Ipv4Addr, TcpStream},
    str::FromStr,
};

use anyhow::anyhow;
use log::{debug, error, info, warn};
use nalgebra::DMatrix;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, UdpSocket},
};

use serde::{Deserialize, Serialize};

use pnet::datalink::{self, NetworkInterface};

use crate::{Action, ShipKind, ShipName, WindData};

const PROTO_IDENTIFIER: u8 = 69;
const CONTROLLER_CLIENT_ID: ShipName = 0;
const CLIENT_REGISTER_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(150);
const CLIENT_LISTEN_PORT: u16 = 6969;
const CLIENT_REJOIN_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);
const CLIENT_HEARTBEAT_TCP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
const CLIENT_HEARTBEAT_TCP_INTERVAL: std::time::Duration = std::time::Duration::from_millis(200);
const SERVER_DROP_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(200);
const CLIENT_TO_CLIENT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(60); // TODO or never?

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum PacketKind {
    Acknowledge,
    JoinRequest(u16, ShipKind),
    Welcome(crate::NetworkShipAddress), // the id of the rat so the coordinator can differentiate them and the tcp port for 1:1 and heartbeat
    Heartbeat,
    Disconnect,
    RawDataf64(nalgebra::DMatrix<f64>), // TODO maybe we need to have a more generic type here but for now it is enough
    RawDataf32(nalgebra::DMatrix<f32>),
    RawDatai32(nalgebra::DMatrix<i32>),
    RawDatau8(nalgebra::DMatrix<u8>),
    VariableTaskRequest(String),
    RatAction(Action),
    Wind(WindData),
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
    pub recv: tokio::sync::broadcast::Sender<Packet>,
    // send to this client
    pub send: tokio::sync::mpsc::Sender<Packet>,
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
                            debug!("Rejoin sent somewhere");
                            to_delete.push(id.clone());
                        }
                    };
                }

                for id in to_delete {
                    new_clients_without_response.remove(&id);
                }
            }
        });

        // task to wait for each new join request
        let clients_tx_inner = clients_tx.clone();
        tokio::spawn(async move {
            loop {
                debug!("waiting for rejoin packages");
                let (addr, packet) = rejoin_req_rx.recv().await.unwrap();
                debug!("got join packet {:?} from {:?}", packet, addr);
                match packet.data {
                    PacketKind::JoinRequest(client_tcp_port, ship_kind) => {
                        let generated_id = rand::random::<ShipName>().abs();
                        let (disconnect_tx, _disconnect_rx) =
                            tokio::sync::broadcast::channel::<bool>(1);

                        // task to handle tcp connection to this client
                        let curr_client_create_sender = clients_tx_inner.clone();
                        let disc_broad = disconnect_tx.clone();
                        tokio::spawn(async move {
                            let tcp_listener = tokio::net::TcpListener::bind("0.0.0.0:0")
                                .await
                                .expect("could not bind tcp socket");

                            let server_tcp_client_listen_port = tcp_listener
                                .local_addr()
                                .expect("could not get tcp listener adress")
                                .port();
                            let tcp_listener =
                                tokio::net::TcpListener::from_std(tcp_listener.into_std().unwrap())
                                    .unwrap();

                            let ip = addr.split(':').next().unwrap();
                            let mut gen_id_send_stream = tokio::net::TcpStream::connect(format!(
                                "{}:{}",
                                ip, client_tcp_port
                            ))
                            .await
                            .expect("could not connect to client");

                            let ip_parsed = Ipv4Addr::from_str(ip).expect("Strange ip format");

                            let client_addr = crate::NetworkShipAddress {
                                ip: ip_parsed.octets(),
                                port: server_tcp_client_listen_port,
                                ship: generated_id,
                            };

                            let welcome_packet = Packet {
                                header: Header {
                                    source: CONTROLLER_CLIENT_ID,
                                    target: generated_id,
                                },
                                data: PacketKind::Welcome(client_addr.clone()),
                            };

                            let welcome_packet_bytes = bincode::serialize(&welcome_packet)
                                .expect("could not serialize welcome packet");
                            let mut welcome_packet = vec![PROTO_IDENTIFIER];
                            welcome_packet.extend(welcome_packet_bytes);
                            match gen_id_send_stream.write_all(&welcome_packet).await {
                                Err(e) => {
                                    error!("could not send welcome packet: {e}");
                                    return;
                                }
                                Ok(_) => {}
                            }

                            let (tx, _) = tokio::sync::broadcast::channel::<Packet>(10);
                            let tx_out = tx.clone();

                            let (client_sender_tx, mut client_sender_rx) =
                                tokio::sync::mpsc::channel::<Packet>(10);

                            // task to handle all packet communication from and to each client
                            let disc_broad_inner = disc_broad.clone();
                            tokio::spawn(async move {
                                let accepted = tcp_listener.accept().await;
                                match accepted {
                                    Err(e) => {
                                        error!("Could not accept Client connection: {e}");
                                    }
                                    Ok((stream, _info)) => {
                                        let (mut rh, mut wh) = stream.into_split(); // TODO inefficient because into but don't know other way right now

                                        let write_disc = disc_broad_inner.clone();
                                        // task for writing from coordinator to client
                                        tokio::spawn(async move {
                                            loop {
                                                let mut disc_broad_inner_loop =
                                                    write_disc.subscribe();
                                                tokio::select! {
                                                    _ = disc_broad_inner_loop.recv() => {
                                                        return;
                                                    }
                                                    msg = client_sender_rx.recv() => {
                                                        match msg {
                                                            None => {
                                                                return; // Channel closed
                                                            }
                                                            Some(msg) => {
                                                                let packet = bincode::serialize(&msg).expect("could not serialize packet");
                                                                 if let Err(e) = wh.write_all(&packet).await {
                                                                     error!("could not respond tcp socket: {e}");
                                                                 }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        });

                                        // task for reading from open client socket
                                        tokio::spawn(async move {
                                            // This streams keeps open for the entirety of the client being connected to the coordinator
                                            let mut buffer = Vec::with_capacity(1024);
                                            let mut first = true;

                                            let mut buf = [0; 1024];
                                            let mut is_for_us = true;
                                            let disc_broad_inner_loop = disc_broad_inner.clone();
                                            loop {
                                                let mut disc_broad_per_conn =
                                                    disc_broad_inner_loop.subscribe();
                                                tokio::select! {
                                                    _ = disc_broad_per_conn.recv() => {
                                                        return;
                                                    }
                                                    read = rh.read(&mut buf) => {
                                                        let n = match read {
                                                            Err(e) => {
                                                                error!(
                                                                    "Could not read from TCP stream: {e}"
                                                                );
                                                                0
                                                            }
                                                            Ok(n) => n,
                                                        };
                                                        if first {
                                                            first = false;
                                                            if buf[0] != PROTO_IDENTIFIER {
                                                                is_for_us = false;
                                                                break;
                                                            }
                                                        }
                                                        if n == 0 {
                                                            break;
                                                        }

                                                        buffer.extend_from_slice(&buf[..n]);
                                                    }
                                                }
                                            }

                                            if is_for_us {
                                                return; // TODO is this just returning from the case in the select?
                                            }
                                            info!("Received stream");
                                            let cleaned_buf = &buffer[1..];
                                            let packet: Packet =
                                                match bincode::deserialize(&cleaned_buf) {
                                                    Err(e) => {
                                                        error!("Received package is broken: {e}");
                                                        return; // TODO Same as above
                                                    }
                                                    Ok(packet) => packet,
                                                };

                                            match tx.send(packet) {
                                                Err(e) => {
                                                    error!(
                                                        "could not send to internal channel: {e}"
                                                    );
                                                }
                                                _ => {}
                                            }
                                        });
                                    }
                                }
                            });

                            let ship_handle = ShipHandle {
                                ship: generated_id,
                                disconnect: disconnect_tx,
                                recv: tx_out,
                                send: client_sender_tx,
                                name: Self::unpad_ship_kind_name(&ship_kind),
                                addr_from_coord: client_addr,
                            };
                            curr_client_create_sender.send(ship_handle).unwrap();
                        });
                    }
                    _ => {
                        warn!("Received unexpected packet: {packet:?}");
                    }
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

    fn pad_ship_kind_name(kind: &ShipKind) -> ShipKind {
        match kind {
            ShipKind::Rat(name) => ShipKind::Rat(Self::pad_string(name)),
            ShipKind::Wind(name) => ShipKind::Wind(Self::pad_string(name)),
            ShipKind::God => ShipKind::God,
        }
    }
    fn unpad_ship_kind_name(kind: &ShipKind) -> ShipKind {
        match kind {
            ShipKind::Rat(name) => ShipKind::Rat(Self::reverse_padding(name)),
            ShipKind::Wind(name) => ShipKind::Wind(Self::reverse_padding(name)),
            ShipKind::God => ShipKind::God,
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

#[derive(Debug)]
pub struct Client {
    heartbeat_task: Option<tokio::task::JoinHandle<()>>,
    tcp_listener_chan: tokio::sync::broadcast::Sender<(Packet, std::net::SocketAddr)>,
    tcp_port: u16,
    pub coordinator_send: Option<tokio::sync::mpsc::Sender<Packet>>,
    pub coordinator_receive: Option<tokio::sync::broadcast::Sender<Packet>>,
    tcp_coordinator_task: Option<tokio::task::JoinHandle<()>>,
    ip: [u8; 4],
    other_client_listener: tokio::net::TcpListener,
    pub other_client_entrance: u16, // tcp port for 1:1 connections
    pub kind: ShipKind,
}

impl Client {
    pub async fn get_udp_socket(external_ip: Option<[u8; 4]>, port: Option<u16>) -> UdpSocket {
        let ip = external_ip.unwrap_or([0, 0, 0, 0]);
        let bind_address = format!(
            "{}.{}.{}.{}:{}",
            ip[0],
            ip[1],
            ip[2],
            ip[3],
            port.unwrap_or(0)
        );
        let socket = std::net::UdpSocket::bind(&bind_address).unwrap();
        socket.set_broadcast(true).expect("could not set broadcast");
        let udp_socket = UdpSocket::from_std(socket).expect("could not promote to tokio socket");
        udp_socket
            .writable()
            .await
            .expect("udp socket not writable");
        udp_socket
    }

    pub async fn init(ship_kind: ShipKind, external_ip: Option<[u8; 4]>) -> Self {
        let ip = external_ip.unwrap_or([0, 0, 0, 0]);
        let tcp_port = if matches!(ship_kind, ShipKind::God) {
            CLIENT_LISTEN_PORT
        } else {
            0
        };
        let bind_address = format!("{}.{}.{}.{}:{}", ip[0], ip[1], ip[2], ip[3], tcp_port);
        let socket = std::net::TcpListener::bind(&bind_address).expect("could not bind tcp socket");
        let tcp_port = socket
            .local_addr()
            .expect("could not get tcp listener adress")
            .port();
        let tcp_listener =
            TcpListener::from_std(socket).expect("could not create tokio tcp listener");

        let bind_address = format!("{}.{}.{}.{}:{}", ip[0], ip[1], ip[2], ip[3], 0); // always find this dynamically
        let client_listener = tokio::net::TcpListener::bind(&bind_address)
            .await
            .expect("could not bind tcp socket");
        let client_listener_port = client_listener
            .local_addr()
            .expect("could not get client listener adress")
            .port();

        let (tx, _) = tokio::sync::broadcast::channel(100);
        let t = tx.clone();
        tokio::spawn(async move {
            loop {
                match tcp_listener.accept().await {
                    Err(e) => {
                        error!("failed to accept tcp connection: {e}");
                    }

                    Ok((mut stream, partner)) => {
                        info!("Established connection with {}", partner);
                        match stream.readable().await {
                            Err(e) => {
                                error!("TCP Listener stream not readable: {e}");
                            }
                            Ok(_) => {}
                        };

                        let mut buffer = Vec::with_capacity(1024);
                        let mut first = true;

                        let mut is_for_us = true;
                        loop {
                            let mut buf = [0; 1024];
                            let n = match stream.read(&mut buf).await {
                                Err(e) => {
                                    error!("Could not read from TCP stream: {e}");
                                    0
                                }
                                Ok(n) => n,
                            };

                            if first {
                                first = false;
                                if buf[0] != PROTO_IDENTIFIER {
                                    is_for_us = false;
                                    break;
                                } else {
                                    let nbuf = [0; 1024];
                                    for i in 0..n {
                                        buf[i] = nbuf[i + 1];
                                    }
                                    buf = nbuf;
                                }
                            }
                            if n == 0 {
                                break;
                            }

                            buffer.extend_from_slice(&buf[..n]);
                        }

                        if is_for_us {
                            continue;
                        }
                        info!("Received stream");
                        let cleaned_buf = &buffer[1..];
                        let packet: Packet = match bincode::deserialize(&cleaned_buf) {
                            Err(e) => {
                                error!("Received package is broken: {e}");
                                continue;
                            }
                            Ok(packet) => packet,
                        };

                        match t.send((packet, partner)) {
                            Err(e) => {
                                error!("TCP Listener could not send to internal channel: {e}");
                            }
                            _ => {}
                        };
                    }
                }
            }
        });

        Self {
            kind: ship_kind,
            other_client_entrance: client_listener_port,
            other_client_listener: client_listener,
            coordinator_send: None,
            coordinator_receive: None,
            heartbeat_task: None,
            tcp_listener_chan: tx,
            tcp_port,
            ip,
            tcp_coordinator_task: None,
        }
    }

    pub fn get_active_interfaces() -> Vec<NetworkInterface> {
        datalink::interfaces()
            .into_iter()
            .filter(|i| i.is_running())
            .collect::<Vec<_>>() // TODO maybe !i.is_loopback()?
    }

    /// Register the client to the network and return a oneshot receiver to wait for the disconnect.
    /// The function will run until a network is established or an error occurs but the error won't be that no network could be found.
    pub async fn register(&mut self) -> anyhow::Result<tokio::sync::oneshot::Receiver<()>> {
        if let Some(coord_tcp) = self.tcp_coordinator_task.as_ref() {
            // the coordinator did not initiat the disconnect, so we need to do it
            if !coord_tcp.is_finished() {
                if let (Some(coord_send), Some(coord_recv)) = (
                    self.coordinator_send.as_ref(),
                    self.coordinator_receive.as_ref(),
                ) {
                    let packet = Packet {
                        header: Header::default(),
                        data: PacketKind::Disconnect,
                    };
                    let mut coord_receiver = coord_recv.subscribe();
                    coord_send.send(packet).await?;
                    let timeouted_disconnect_answer = tokio::time::timeout(
                        std::time::Duration::from_secs(1),
                        coord_receiver.recv(),
                    )
                    .await;
                    match timeouted_disconnect_answer {
                        Ok(Ok(packet)) => {
                            if !matches!(packet.data, PacketKind::Acknowledge) {
                                return Err(anyhow!("Received unexpected packet: {packet:?}"));
                            }
                        }
                        Ok(Err(e)) => {
                            return Err(anyhow!("Could not receive disconnect acknowledge: {e}"));
                        }
                        Err(_) => {
                            return Err(anyhow!("Disconnect acknowledge timed out"));
                        }
                    }
                } else {
                    return Err(anyhow!("coordinator sender and/or receiver does not exist"));
                }
            }

            self.heartbeat_task.take().map(|t| t.abort());
            self.tcp_coordinator_task.take().map(|t| t.abort());
            self.coordinator_receive.take();
            self.coordinator_send.take();
        }

        let network_register_packet = Packet {
            header: Header {
                source: ShipName::MAX,
                target: CONTROLLER_CLIENT_ID,
            },
            data: PacketKind::JoinRequest(self.tcp_port, Sea::pad_ship_kind_name(&self.kind)),
        };

        let udp_socket = Self::get_udp_socket(Some(self.ip), None).await;
        let mut subscription = self.tcp_listener_chan.subscribe();

        let (partner, my_addr) = loop {
            Self::send_packet_broadcast(&network_register_packet, &udp_socket).await?;

            let tim = tokio::time::timeout(CLIENT_REGISTER_TIMEOUT, subscription.recv()).await;

            match tim {
                Err(_) => {
                    info!("JoinRequest timed out");
                    continue;
                }
                Ok(Err(e)) => {
                    return Err(anyhow!("TCP channel closed: {e}"));
                }
                Ok(Ok((packet, partner))) => {
                    if let PacketKind::Welcome(addr) = packet.data {
                        break (partner, addr);
                    }
                }
            }
        };

        // each tcp packet must to through the coord_tx, so we can have a heartbeat in between
        let (coord_tx, mut coord_rx) = tokio::sync::mpsc::channel::<Packet>(10);

        // receiver for packets over tcp from coordinator, subscribe if interested.
        let (coord_packet_receiver_tx, _) = tokio::sync::broadcast::channel(10);

        // clone for packet sender from tcp connection
        let coord_packet_receiver_tx_in = coord_packet_receiver_tx.clone();

        let (coord_raw_tx, mut coord_raw_rx) = tokio::sync::mpsc::channel(10);

        let (disconnect_tx, disconnect_rx) = tokio::sync::oneshot::channel();

        // Heartbeat interval if no message send to coordinator
        let heartbeat_task = tokio::spawn(async move {
            let mut heartbeat_interval = tokio::time::interval(CLIENT_REJOIN_POLL_INTERVAL);
            loop {
                tokio::select! {
                    packet = coord_rx.recv() => {
                        if let Some(mut packet) = packet {
                            heartbeat_interval.reset();
                            packet.header.source = my_addr.ship; // overwrite source with our id
                            packet.header.target = CONTROLLER_CLIENT_ID; // overwrite target with controller id
                            match coord_raw_tx.send(packet).await {
                                Err(e) => {
                                    error!("could not forward coordinator packet: {e}");
                                }
                                Ok(_) => {}
                            }
                        }
                    }

                    _ = heartbeat_interval.tick() => {
                        let packet = Packet {
                            header: Header {
                                source: my_addr.ship,
                                target: CONTROLLER_CLIENT_ID,
                            },
                            data: PacketKind::Heartbeat,
                        };
                        match coord_raw_tx.send(packet).await {
                            Err(e) => {
                                error!("could not send heartbeat to coordinator: {e}");
                            }
                            Ok(_) => {}
                        };
                   }
                }
            }
        });

        let stream =
            std::net::TcpStream::connect(format!("{}:{}", partner.ip().to_string(), my_addr.port))?;

        info!("Connected to coordinator!");

        let socket = socket2::Socket::from(stream);
        socket.set_keepalive(true)?;

        socket.set_tcp_keepalive(
            &socket2::TcpKeepalive::new()
                .with_time(CLIENT_HEARTBEAT_TCP_TIMEOUT)
                .with_interval(CLIENT_HEARTBEAT_TCP_INTERVAL),
        )?;

        let stream: TcpStream = socket.into();
        let mut stream = tokio::net::TcpStream::from_std(stream).unwrap();

        let coord_tcp_task = tokio::spawn(async move {
            let mut buffer = Vec::with_capacity(1024);
            stream.readable().await.unwrap();
            stream.writable().await.unwrap();
            let mut buf = [0; 1024];

            loop {
                tokio::select! {
                    packet = coord_raw_rx.recv() => {
                        if let Some(packet) = packet {
                            let packet = bincode::serialize(&packet).expect("could not serialize packet");
                            if let Err(e) = stream.write_all(&packet).await {
                                error!("could not send to tcp socket: {e}");
                            }
                        }

                    }
                    read = stream.read(&mut buf) => {
                        match read {
                            Ok(n) => {
                                let mut first = true;
                                let mut is_for_us = true;

                                loop {
                                    if first {
                                        first = false;
                                        if buf[0] != PROTO_IDENTIFIER {
                                            is_for_us = false;
                                            break;
                                        }
                                    }
                                    if n == 0 {
                                        break;
                                    }

                                    buffer.extend_from_slice(&buf[..n]);
                                    buf.fill(0);
                                }

                                if !is_for_us {
                                    continue;
                                }

                                info!("Received stream");
                                let cleaned_buf = &buffer[1..];
                                let packet: Packet = match bincode::deserialize(&cleaned_buf) {
                                    Err(e) => {
                                        error!("Received package is broken: {e}");
                                        continue;
                                    }
                                    Ok(packet) => packet,
                                };
                                coord_packet_receiver_tx_in.send(packet).unwrap();
                            }
                            Err(e) => {
                                error!("could not read from tcp stream. Notifying channel. {e}");
                                disconnect_tx.send(()).unwrap();
                                return; // stopping task here
                            }
                        }
                    }
                }
            }
        });

        self.coordinator_send = Some(coord_tx);
        self.coordinator_receive = Some(coord_packet_receiver_tx);
        self.heartbeat_task = Some(heartbeat_task);
        self.tcp_coordinator_task = Some(coord_tcp_task);

        Ok(disconnect_rx)
    }

    pub async fn send_raw_to_other_client(
        &self,
        address: &IpAddr,
        port: u16,
        data: Vec<u8>,
    ) -> anyhow::Result<()> {
        let initial_connection_timeout = std::time::Duration::from_secs(1);
        // try for max 1 second to connect to the other client
        let stream = tokio::time::timeout(
            initial_connection_timeout.clone(),
            tokio::net::TcpStream::connect(format!("{}:{}", address.to_string(), port)),
        )
        .await;

        match stream {
            Err(_) => {
                return Err(anyhow!(
                    "Could not connect to other client within {initial_connection_timeout:?}"
                ));
            }
            Ok(Err(e)) => {
                return Err(anyhow!("Could not connect to other client: {e}"));
            }
            Ok(Ok(stream)) => {
                let mut stream = stream;
                stream.writable().await?;
                stream.write_all(&data).await?;
            }
        }

        Ok(())
    }

    pub async fn recv_raw_from_other_client(
        &self,
        sender: Option<&crate::NetworkShipAddress>,
    ) -> anyhow::Result<Vec<u8>> {
        loop {
            let stream = tokio::time::timeout(
                CLIENT_TO_CLIENT_TIMEOUT,
                self.other_client_listener.accept(),
            )
            .await;

            match stream {
                Err(_) => {
                    return Err(anyhow!(
                        "Could not connect to other client within {CLIENT_TO_CLIENT_TIMEOUT:?}"
                    ));
                }
                Ok(Err(e)) => {
                    return Err(anyhow!("Could not connect to other client: {e}"));
                }
                Ok(Ok((mut stream, socket_sender))) => {
                    if let Some(expected_sender) = sender {
                        let expected_ip = Ipv4Addr::new(
                            expected_sender.ip[0],
                            expected_sender.ip[1],
                            expected_sender.ip[2],
                            expected_sender.ip[3],
                        );
                        if expected_ip != socket_sender.ip().to_canonical() {
                            continue;
                        }
                    }
                    stream.readable().await?;
                    let mut buffer = Vec::with_capacity(1024);
                    let mut first = true;

                    let mut is_for_us = true;
                    loop {
                        let mut buf = [0; 1024];
                        let n = match stream.read(&mut buf).await {
                            Err(e) => {
                                error!("Could not read from TCP stream: {e}");
                                0
                            }
                            Ok(n) => n,
                        };

                        if first {
                            first = false;
                            if buf[0] != PROTO_IDENTIFIER {
                                is_for_us = false;
                                break;
                            }
                        }
                        if n == 0 {
                            break;
                        }

                        buffer.extend_from_slice(&buf[..n]);
                    }

                    if is_for_us {
                        return Ok(buffer);
                    } else {
                        return Err(anyhow!("Received data is not for us"));
                    }
                }
            }
        }
    }

    // TODO log to coordinator functions. They send them async in another thread to the coordinator so nothing is blocked on the client side.

    // --- end send/recv ---

    fn calculate_broadcast(ipv4_addr: std::net::Ipv4Addr, prefix_len: u8) -> std::net::Ipv4Addr {
        let mask = !0u32 >> prefix_len;
        let bcast_addr = u32::from(ipv4_addr) | mask;
        std::net::Ipv4Addr::from(bcast_addr)
    }

    async fn send_packet_broadcast(packet: &Packet, udp_socket: &UdpSocket) -> anyhow::Result<()> {
        let raw = bincode::serialize(packet).expect("packet not serializable");
        let mut data = vec![PROTO_IDENTIFIER];
        data.extend(raw);
        for interface in Self::get_active_interfaces().iter() {
            for ip_network in &interface.ips {
                if let IpAddr::V4(ipv4_addr) = ip_network.ip() {
                    // let addr = if ipv4_addr.is_loopback() {
                    // Self::calculate_broadcast(ipv4_addr, ip_network.prefix())
                    // } else {
                    let addr = Self::calculate_broadcast(ipv4_addr, ip_network.prefix());
                    // };

                    let target_str = format!(
                        "{}.{}.{}.{}:{}",
                        addr.octets()[0],
                        addr.octets()[1],
                        addr.octets()[2],
                        addr.octets()[3],
                        CLIENT_LISTEN_PORT
                    );

                    let sent = udp_socket.send_to(&data, &target_str).await?;
                    if sent != data.len() {
                        return Err(anyhow!("data len mismatch with actual send bytes"));
                    }
                }
            }
        }

        Ok(())
    }
}
