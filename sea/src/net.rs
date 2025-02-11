use std::{
    collections::HashMap,
    net::{IpAddr, TcpStream},
};

use anyhow::{anyhow, Error};
use log::{error, info, warn};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpSocket, UdpSocket},
};

use serde::{Deserialize, Serialize};

use pnet::datalink::{self, NetworkInterface};

use crate::ShipName;

const PROTO_IDENTIFIER: u8 = 69;
const CONTROLLER_CLIENT_ID: ShipName = 0;
const CLIENT_REGISTER_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(150);
const CLIENT_LISTEN_PORT: u16 = 6969;
const BI_CLIENT_LISTEN_PORT: u16 = 6970;
const CLIENT_REJOIN_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);
const CLIENT_HEARTBEAT_INTERVAL: std::time::Duration = std::time::Duration::from_millis(500);
const CLIENT_HEARTBEAT_TCP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);
const CLIENT_HEARTBEAT_TCP_INTERVAL: std::time::Duration = std::time::Duration::from_millis(200);
const SERVER_DROP_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(200);

#[derive(Serialize, Deserialize, Clone, Debug)]
enum PacketKind {
    Acknowledge,
    JoinRequest(u16),
    Welcome(ShipName, u16), // the id of the rat so the coordinator can differentiate them and the tcp port for 1:1 and heartbeat
    Heartbeat,
    Disconnect,
    RawDataf64(nalgebra::DMatrix<f64>), // TODO maybe we need to have a more generic type here but for now it is enough
    RawDataf32(nalgebra::DMatrix<f32>),
    RawDatai32(nalgebra::DMatrix<i32>),
}

pub trait SeaSendableScalar: nalgebra::Scalar {}
impl SeaSendableScalar for f64 {}
impl SeaSendableScalar for f32 {}
impl SeaSendableScalar for i32 {}

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
struct Header {
    pub source: ShipName,
    pub target: ShipName,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
struct Packet {
    header: Header,
    data: PacketKind,
}

#[derive(Clone, Debug)]
struct ShipHandle {
    ship: ShipName,
    // Send here to disconnect the tcp listener
    disconnect: tokio::sync::broadcast::Sender<bool>,
    // get requests from the client and send response back via the sender
    req_resp:
        std::sync::Arc<tokio::sync::mpsc::Receiver<(Packet, tokio::sync::mpsc::Sender<Packet>)>>,
}

#[derive(Debug)]
pub struct Sea {
    // coordinator_client: Client,
    network_clients_chan: tokio::sync::broadcast::Sender<ShipHandle>,
    dissolve_network: tokio::sync::mpsc::Sender<tokio::sync::mpsc::Sender<()>>,
}

/// Server handling the Sea network
impl Sea {
    pub async fn init(
        external_ip: Option<[u8; 4]>,
        rules: &HashMap<String, crate::VarPair>,
    ) -> Self {
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
            let udp_listener = Client::get_udp_socket(external_ip).await;
            let rejoin_request = Packet {
                header: Header {
                    source: ShipName::MAX,
                    target: CONTROLLER_CLIENT_ID,
                },
                data: PacketKind::JoinRequest(0),
            };
            let bytes_rejoin_request =
                bincode::serialize(&rejoin_request).expect("could not serialize rejoin request");
            let expected_n_bytes_for_rejoin_request = bytes_rejoin_request.len() + 1;
            let mut new_clients_without_response = HashMap::<String, (usize, Vec<u8>)>::new();

            loop {
                let mut buf = [0; 1024];
                let (n, addr) = udp_listener.recv_from(&mut buf).await.unwrap();
                let id = format!("{}:{}", addr.ip(), addr.port());

                match new_clients_without_response.get_mut(&id) {
                    Some((kum, buffer)) => {
                        if *kum == 0 {
                            if buf[0] != PROTO_IDENTIFIER {
                                new_clients_without_response.remove(&id); // TODO does this work? We are inside a get_mut but removing the id we are in
                                continue;
                            } else {
                                let nbuf = [0; 1024];
                                // clean the first byte from the buffer
                                for i in 0..n {
                                    buf[i] = nbuf[i + 1];
                                }
                                buf = nbuf;

                                buffer.extend_from_slice(&buf[..n]);
                            }
                        }
                        *kum += n;
                    }
                    None => {
                        let mut buffer = Vec::with_capacity(1024);
                        buffer.extend_from_slice(&buf[..n]);
                        new_clients_without_response.insert(id, (n, buffer));
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
            }
        });

        // task to wait for each new join request
        let clients_tx_inner = clients_tx.clone();
        tokio::spawn(async move {
            loop {
                let (addr, packet) = rejoin_req_rx.recv().await.unwrap();
                match packet.data {
                    PacketKind::JoinRequest(client_tcp_port) => {
                        let generated_id = rand::random::<ShipName>().abs();
                        let (disconnect_tx, mut disconnect_rx) =
                            tokio::sync::broadcast::channel::<bool>(1);

                        // task to handle tcp connection to this client
                        let curr_client_create_sender = clients_tx_inner.clone();
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

                            // send welcome with the generated client_id
                            let welcome_packet = Packet {
                                header: Header {
                                    source: CONTROLLER_CLIENT_ID,
                                    target: generated_id,
                                },
                                data: PacketKind::Welcome(
                                    generated_id,
                                    server_tcp_client_listen_port,
                                ),
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

                            let (tx, rx) = tokio::sync::mpsc::channel::<(
                                Packet,
                                tokio::sync::mpsc::Sender<Packet>,
                            )>(10);

                            let (client_sender_tx, mut client_sender_rx) =
                                tokio::sync::mpsc::channel::<Packet>(10);

                            // task to handle all packet communication from and to each client
                            let client_answer_sender = client_sender_tx.clone();
                            tokio::spawn(async move {
                                let accepted = tcp_listener.accept().await;
                                match accepted {
                                    Err(e) => {
                                        error!("Could not accept Client connection: {e}");
                                    }
                                    Ok((stream, _info)) => {
                                        let (mut rh, wh) = stream.into_split(); // TODO inefficient because into but don't know other way right now

                                        // TODO write on received client_sender_rx
                                        tokio::spawn(async move {});

                                        // TODO tokio select over read and disconnect
                                        tokio::spawn(async move {
                                            // This streams keeps open for the entirety of the client being connected to the coordinator
                                            let mut buffer = Vec::with_capacity(1024);
                                            let mut first = true;

                                            let mut buf = [0; 1024];
                                            let mut is_for_us = true;
                                            loop {
                                                tokio::select! {
                                                    _ = disconnect_rx.recv() => {
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

                                            let (response_tx, mut response_rx) =
                                                tokio::sync::mpsc::channel(1);
                                            match tx.send((packet, response_tx)).await {
                                                Err(e) => {
                                                    error!(
                                                        "could not send to internal channel: {e}"
                                                    );
                                                }
                                                _ => {}
                                            }

                                            if let Some(response) = response_rx.recv().await {
                                                client_answer_sender.send(response).await.unwrap();
                                                // let packet = bincode::serialize(&response).expect("could not serialize packet");
                                                // if let Err(e) = stream.write_all(&packet).await {
                                                // error!("could not respond tcp socket: {e}");
                                                // }
                                            }
                                        });
                                    }
                                }

                                loop {
                                    tokio::select! {
                                        _ = disconnect_rx.recv() => {
                                            return;
                                        }
                                        accepted = tcp_listener.accept() => {
                                            match accepted {
                                                Ok((stream, _addr)) => {
                                                    // This streams keeps open for the entirety of the client being connected to the coordinator
                                                    let mut buffer = Vec::with_capacity(1024);
                                                    let mut first = true;

                                                    let (mut rh, wh) = stream.into_split(); // TODO inefficient because into but don't know other way right now

                                                    tokio::spawn(async move {
                                                        while let Some(msg) = client_sender_rx.recv().await {

                                                        }
                                                    });

                                                    let mut is_for_us = true;
                                                    loop {
                                                        let mut buf = [0; 1024];
                                                        let n = match rh.read(&mut buf).await {
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
                                                        return; // TODO is this just returning from the case in the select?
                                                    }
                                                    info!("Received stream");
                                                    let cleaned_buf = &buffer[1..];
                                                    let packet: Packet = match bincode::deserialize(&cleaned_buf) {
                                                        Err(e) => {
                                                            error!("Received package is broken: {e}");
                                                            return; // TODO Same as above
                                                        }
                                                        Ok(packet) => packet,
                                                    };

                                                    let (response_tx, mut response_rx) = tokio::sync::mpsc::channel(1);
                                                    match tx.send((packet, response_tx)).await {
                                                        Err(e) => {
                                                            error!("could not send to internal channel: {e}");
                                                        }
                                                        _ => {}
                                                    }

                                                    if let Some(response) = response_rx.recv().await {
                                                        client_answer_sender.send(response).await.unwrap();
                                                        // let packet = bincode::serialize(&response).expect("could not serialize packet");
                                                        // if let Err(e) = stream.write_all(&packet).await {
                                                            // error!("could not respond tcp socket: {e}");
                                                        // }

                                                    }
                                                }
                                                Err(e) => {
                                                    error!("Error while receiving from client: {e}");
                                                }

                                            }

                                        }
                                    }
                                }
                            });

                            let ship_handle = ShipHandle {
                                ship: generated_id,
                                disconnect: disconnect_tx,
                                req_resp: std::sync::Arc::new(rx),
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

        // TODO mak

        // TODO enforce the rules by listening to all new clients and checking what they send.
        tokio::spawn(async move {
            // clients_tx.
        });

        // TODO the coordinator needs clients because the coordinator can not send from itself to the wind clients.
        // But the client can ask the coordinator. So if we do a flag or special id for the coordinator client (0),
        // we can set variables or bacon to ros messages where each ros message has a separate client. As soon as the reader code toggles the variable,
        // it asks the coordinator what to do. it then says to sync it with one or more
        Self {
            network_clients_chan: clients_tx,
            // coordinator_client: Client::init(true, external_ip).await,
            dissolve_network: dissolve_network_tx,
        }
    }
}

impl Drop for Sea {
    fn drop(&mut self) {
        tokio::runtime::Handle::current().block_on(async move {
            let (answer_tx, mut answer_rx) = tokio::sync::mpsc::channel(1);
            match self.dissolve_network.send(answer_tx).await {
                Err(e) => {
                    error!("Error while droppping network: {e}");
                }
                Ok(_) => {}
            }

            let answer_timeout = tokio::time::timeout(SERVER_DROP_TIMEOUT, answer_rx.recv());
            match answer_timeout.await {
                Err(e) => {
                    error!("Dropping network timeout, discarding waiting for completion: {e}");
                }
                Ok(None) => {
                    error!("Sender already closed in dissolving answer");
                }
                _ => {}
            }
        });
    }
}

#[derive(Debug)]
pub struct Client {
    heartbeat_task: Option<tokio::task::JoinHandle<()>>,
    tcp_listen_task: tokio::task::JoinHandle<()>,
    pub tcp_listener_chan: tokio::sync::broadcast::Sender<(Packet, std::net::SocketAddr)>,
    tcp_port: u16,
    coordinator_send: Option<tokio::sync::mpsc::Sender<Packet>>,
    coordinator_receive: Option<tokio::sync::broadcast::Sender<Packet>>,
    tcp_coordinator_task: Option<tokio::task::JoinHandle<()>>,
    ip: [u8; 4],
    other_client_listener: tokio::net::TcpListener,
    pub other_client_entrance: u16, // tcp port for 1:1 connections
}

#[derive(Debug, Clone, Copy)]
struct NetworkState {
    pub coordinator_tcp_port: u16,
    pub my_network_id: ShipName,
    pub coordinator_ip: IpAddr,
}

impl Client {
    pub async fn get_udp_socket(external_ip: Option<[u8; 4]>) -> UdpSocket {
        let ip = external_ip.unwrap_or([0, 0, 0, 0]);
        let bind_address = format!("{}.{}.{}.{}:0", ip[0], ip[1], ip[2], ip[3]);
        let socket = std::net::UdpSocket::bind(&bind_address).unwrap();
        /*let udp_port = socket
        .local_addr()
        .expect("could not get sock adress")
        .port();*/
        socket.set_broadcast(true).expect("could not set broadcast");
        let udp_socket = UdpSocket::from_std(socket).expect("could not promote to tokio socket");
        udp_socket
            .writable()
            .await
            .expect("udp socket not writable");
        udp_socket
    }

    pub async fn init(controller: bool, external_ip: Option<[u8; 4]>) -> Self {
        let ip = external_ip.unwrap_or([0, 0, 0, 0]);
        let tcp_port = if controller { CLIENT_LISTEN_PORT } else { 0 };
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
        let client_tcp_listen_task = tokio::spawn(async move {
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
            other_client_entrance: client_listener_port,
            other_client_listener: client_listener,
            coordinator_send: None,
            coordinator_receive: None,
            heartbeat_task: None,
            tcp_listener_chan: tx,
            tcp_listen_task: client_tcp_listen_task,
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
            data: PacketKind::JoinRequest(self.tcp_port),
        };

        let udp_socket = Self::get_udp_socket(Some(self.ip)).await;
        let mut subscription = self.tcp_listener_chan.subscribe();

        let (partner, my_id, coordinator_tcp_port) = loop {
            Self::send_packet_broadcast(&network_register_packet, &udp_socket).await?;

            let tim = tokio::time::timeout(CLIENT_REGISTER_TIMEOUT, subscription.recv()).await;

            match tim {
                Err(_) => {
                    info!("JoingRequest timed out");
                    continue;
                }
                Ok(Err(e)) => {
                    return Err(anyhow!("TCP channel closed: {e}"));
                }
                Ok(Ok((packet, partner))) => {
                    if let PacketKind::Welcome(my_id, coordinator_tcp_port) = packet.data {
                        break (partner, my_id, coordinator_tcp_port);
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
                            packet.header.source = my_id; // overwrite source with our id
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
                                source: my_id,
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

        let stream = std::net::TcpStream::connect(format!(
            "{}:{}",
            partner.ip().to_string(),
            coordinator_tcp_port
        ))?;

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

    async fn send_raw_to_other_client(
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

    async fn recv_raw_from_other_client(&self) -> anyhow::Result<Vec<u8>> {
        let initial_connection_timeout = std::time::Duration::from_secs(1);

        let stream = tokio::time::timeout(
            initial_connection_timeout.clone(),
            self.other_client_listener.accept(),
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
            Ok(Ok((mut stream, _))) => {
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

    // TODO log to coordinator functions. They send them async in another thread to the coordinator so nothing is blocked on the client side.

    // --- send ---
    pub async fn send_to_other_client_f64(
        &self,
        address: &IpAddr,
        port: u16,
        data: nalgebra::DMatrix<f64>,
    ) -> anyhow::Result<()> {
        let raw_data =
            bincode::serialize(&PacketKind::RawDataf64(data)).expect("data not serializable");
        self.send_raw_to_other_client(address, port, raw_data).await
    }

    pub async fn send_to_other_client_f32(
        &self,
        address: &IpAddr,
        port: u16,
        data: nalgebra::DMatrix<f32>,
    ) -> anyhow::Result<()> {
        let raw_data =
            bincode::serialize(&PacketKind::RawDataf32(data)).expect("data not serializable");
        self.send_raw_to_other_client(address, port, raw_data).await
    }

    pub async fn send_to_other_client_i32(
        &self,
        address: &IpAddr,
        port: u16,
        data: nalgebra::DMatrix<i32>,
    ) -> anyhow::Result<()> {
        let raw_data =
            bincode::serialize(&PacketKind::RawDatai32(data)).expect("data not serializable");
        self.send_raw_to_other_client(address, port, raw_data).await
    }

    // --- recv ---
    pub async fn recv_from_other_client_f64(&self) -> anyhow::Result<nalgebra::DMatrix<f64>> {
        let raw_data = self.recv_raw_from_other_client().await?;
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDataf64(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }

    pub async fn recv_from_other_client_f32(&self) -> anyhow::Result<nalgebra::DMatrix<f32>> {
        let raw_data = self.recv_raw_from_other_client().await?;
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDataf32(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }

    pub async fn recv_from_other_client_i32(&self) -> anyhow::Result<nalgebra::DMatrix<i32>> {
        let raw_data = self.recv_raw_from_other_client().await?;
        let data: PacketKind = bincode::deserialize(&raw_data).expect("data not deserializable");
        match data {
            PacketKind::RawDatai32(data) => Ok(data),
            _ => Err(anyhow!("Received wrong data type")),
        }
    }

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
                    let broadcast = Self::calculate_broadcast(ipv4_addr, ip_network.prefix());

                    let target_str = format!(
                        "{}.{}.{}.{}:{}",
                        broadcast.octets()[0],
                        broadcast.octets()[1],
                        broadcast.octets()[2],
                        broadcast.octets()[3],
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
