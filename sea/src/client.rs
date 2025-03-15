use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream};

use anyhow::anyhow;
use log::{debug, error, info, warn};
use pnet::datalink::{self, NetworkInterface};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{
        tcp::{OwnedReadHalf, OwnedWriteHalf},
        TcpListener, UdpSocket,
    },
};

use crate::{
    coordinator::COMPARE_NODE_NAME,
    net::{
        Header, Packet, PacketKind, Sea, CLIENT_HEARTBEAT_TCP_INTERVAL,
        CLIENT_HEARTBEAT_TCP_TIMEOUT, CLIENT_LISTEN_PORT, CLIENT_REGISTER_TIMEOUT,
        CLIENT_REJOIN_POLL_INTERVAL, CLIENT_TO_CLIENT_INIT_RETRY_TIMEOUT, CLIENT_TO_CLIENT_TIMEOUT,
        CONTROLLER_CLIENT_ID, PROTO_IDENTIFIER,
    },
    ShipKind, ShipName, VariableType,
};

#[derive(Debug)]
pub struct Client {
    pub coordinator_receive: std::sync::Arc<
        std::sync::RwLock<
            Option<tokio::sync::broadcast::Sender<(Packet, Option<std::net::SocketAddr>)>>,
        >,
    >,
    tcp_port: u16,
    pub coordinator_send:
        std::sync::Arc<std::sync::RwLock<Option<tokio::sync::mpsc::Sender<Packet>>>>,
    ip: [u8; 4],
    other_client_listener: tokio::net::TcpListener,
    pub other_client_entrance: u16, // tcp port for 1:1 connections
    pub kind: ShipKind,
    connected: tokio::sync::mpsc::Receiver<bool>,
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
        let bind_address = format!("{}.{}.{}.{}:{}", ip[0], ip[1], ip[2], ip[3], 0);
        // TCP socket for Coordinator communication
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

        let (connected_tx, connected_rx) = tokio::sync::mpsc::channel(1);

        let coord_raw_send_tx = std::sync::Arc::new(std::sync::RwLock::new(None));
        let coord_raw_receive_tx: std::sync::Arc<
            std::sync::RwLock<Option<tokio::sync::broadcast::Sender<(Packet, Option<SocketAddr>)>>>,
        > = std::sync::Arc::new(std::sync::RwLock::new(None));
        let coord_raw_send_tx_l = coord_raw_send_tx.clone();
        let coord_raw_receive_tx_l = coord_raw_receive_tx.clone();
        tokio::spawn(async move {
            loop {
                match tcp_listener.accept().await {
                    Err(e) => {
                        error!("failed to accept tcp connection: {e}");
                    }

                    Ok((stream, partner)) => {
                        // each tcp packet from outside logic must go through the coord_tx, so we can have a heartbeat in between. The results are sent to the raw
                        let (coord_send_tx_inner, mut coord_send_rx_inner) =
                            tokio::sync::mpsc::channel::<Packet>(10);

                        // the raw sender is used by the socket.
                        let (coord_raw_send_tx_inner, coord_raw_send_rx_inner) =
                            tokio::sync::mpsc::channel::<Packet>(10);

                        // receiver for communication as broadcast. subscribe to receive from outside
                        let (coord_raw_receive_tx_inner, _) = tokio::sync::broadcast::channel(10);

                        let socket = socket2::Socket::from(stream.into_std().unwrap());
                        socket.set_keepalive(true).unwrap();
                        socket
                            .set_tcp_keepalive(
                                &socket2::TcpKeepalive::new()
                                    .with_time(CLIENT_HEARTBEAT_TCP_TIMEOUT)
                                    .with_interval(CLIENT_HEARTBEAT_TCP_INTERVAL),
                            )
                            .unwrap();
                        let stream: TcpStream = socket.into();
                        let stream = tokio::net::TcpStream::from_std(stream).unwrap();
                        match stream.readable().await {
                            Err(e) => {
                                error!("TCP Listener stream not readable: {e}");
                                continue;
                            }
                            Ok(_) => {}
                        };

                        let (rh, wh) = stream.into_split();

                        // sending packets to coordinator through the TCP socket
                        tokio::spawn(async move {
                            Self::send_to_socket(coord_raw_send_rx_inner, wh).await;
                        });

                        // receiving packets from coordinator through the TCP socket
                        let coord_raw_rx_tx = coord_raw_receive_tx_inner.clone();
                        tokio::spawn(async move {
                            Self::receive_from_socket(rh, coord_raw_rx_tx, Some(partner)).await;
                        });

                        let mut coord_heartbeat_sub = coord_raw_receive_tx_inner.subscribe();
                        // tokio::task::yield_now().await; // not receiving welcome packet in lower spawn when not yielding here

                        let coord_raw_sender = coord_raw_send_tx_inner.clone();
                        let connected_tx_inner = connected_tx.clone();
                        let coord_raw_receive = coord_raw_receive_tx_l.clone();
                        let coord_raw_send = coord_raw_send_tx_l.clone();
                        // wait for the welcome packet and when received, start heartbeat for as long as the write channel is open
                        while let Ok((packet, _partner)) = coord_heartbeat_sub.recv().await {
                            if let PacketKind::Welcome(my_addr) = packet.data {
                                // TODO maybe send the coord_send_tx_inner directly if someone wants to use it
                                connected_tx_inner.send(true).await.unwrap();
                                {
                                    // connection accepted here, so we can replace the sender channel
                                    coord_raw_send
                                        .write()
                                        .unwrap()
                                        .replace(coord_send_tx_inner.clone());
                                    // ... and the receiver as well
                                    coord_raw_receive
                                        .write()
                                        .unwrap()
                                        .replace(coord_raw_receive_tx_inner.clone());
                                }

                                // setting up heartbeats
                                loop {
                                    let timeout = tokio::time::timeout(
                                        CLIENT_REJOIN_POLL_INTERVAL,
                                        coord_send_rx_inner.recv(),
                                    )
                                    .await;
                                    match timeout {
                                        Ok(Some(mut packet)) => {
                                            packet.header.source = my_addr.ship; // overwrite source with our id
                                            packet.header.target = CONTROLLER_CLIENT_ID; // overwrite target with controller id
                                            match coord_raw_sender.send(packet).await {
                                                Err(e) => {
                                                    error!(
                                                        "could not forward coordinator packet: {e}"
                                                    );
                                                }
                                                Ok(_) => {
                                                    debug!("send packet");
                                                }
                                            }
                                        }
                                        Err(_e) => {
                                            let packet = Packet {
                                                header: crate::net::Header {
                                                    source: my_addr.ship,
                                                    target: CONTROLLER_CLIENT_ID,
                                                },
                                                data: PacketKind::Heartbeat,
                                            };
                                            match coord_raw_sender.send(packet).await {
                                                Err(e) => {
                                                    error!("could not send heartbeat to coordinator: {e}");
                                                    return; // channel closed
                                                }
                                                Ok(_) => {
                                                    // debug!("sending heartbeat");
                                                }
                                            };
                                        }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });

        Self {
            kind: ship_kind,
            other_client_entrance: client_listener_port,
            other_client_listener: client_listener,
            coordinator_send: coord_raw_send_tx,
            coordinator_receive: coord_raw_receive_tx,
            connected: connected_rx,
            tcp_port,
            ip,
        }
    }

    pub async fn send_to_socket(
        mut channel: tokio::sync::mpsc::Receiver<Packet>,
        mut wh: OwnedWriteHalf,
    ) {
        while let Some(packet) = channel.recv().await {
            // let orig_packet = packet.clone(); // TODO rm after dbg
            let packet = bincode::serialize(&packet).expect("could not serialize packet");
            let packet_size = packet.len() as u32;
            let packet_size_buffer = packet_size.to_be_bytes();
            let mut buffer = vec![
                PROTO_IDENTIFIER,
                packet_size_buffer[0],
                packet_size_buffer[1],
                packet_size_buffer[2],
                packet_size_buffer[3],
            ];
            buffer.extend_from_slice(&packet);
            // debug!("writing buffer: {}, {}", buffer.len(), packet.len());
            if let Err(e) = wh.write_all(&buffer).await {
                error!("could not send to tcp socket: {e}");
                channel.close();
                return;
            }
            // debug!("written packet to tcp stream {:?}", orig_packet);
        }
    }

    pub async fn receive_from_socket(
        mut rh: OwnedReadHalf,
        channel: tokio::sync::broadcast::Sender<(Packet, Option<SocketAddr>)>,
        partner: Option<SocketAddr>,
    ) {
        const COMM_HEADER_BYTES_N: usize = 5;
        loop {
            tokio::task::yield_now().await;
            let mut buffer = Vec::with_capacity(1024);
            let mut first = true;
            let mut msg_size = 0;

            // collect parts to complete a single packet
            let mut is_for_us = true;
            loop {
                debug!("reading");
                let mut buf = [0; 1024];
                let n = match rh.read(&mut buf).await {
                    Err(e) => {
                        error!("Could not read from TCP stream in client connection to coordinator: {e}");
                        return;
                    }
                    Ok(n) => n,
                };

                if n == 0 {
                    return;
                }

                debug!("reading from buffer {n}");
                // TODO test very large packets that do not fit into the 1024 buffer

                // handles multiple packets in single buffer
                let mut cursor = 0;
                loop {
                    if cursor >= buf.len() || cursor >= n {
                        break;
                    }
                    let max_buf;
                    if first {
                        if buf[cursor] != PROTO_IDENTIFIER {
                            debug!("detected not for us at cursor {}", cursor);
                            is_for_us = false;
                            break;
                        } else {
                            is_for_us = true;
                            cursor += COMM_HEADER_BYTES_N;
                            let msg_size_buf = [buf[1], buf[2], buf[3], buf[4]];
                            // TODO usize vs u32 sizes?
                            msg_size = u32::from_be_bytes(msg_size_buf) as usize;
                            max_buf = std::cmp::min(msg_size + cursor, n);
                            first = false;
                            buffer.extend_from_slice(&buf[cursor..max_buf]);
                        }
                    } else {
                        max_buf = std::cmp::min(msg_size + cursor, n);
                        buffer.extend_from_slice(&buf[cursor..max_buf]);
                    }
                    cursor = max_buf;
                    if msg_size == buffer.len() {
                        debug!("got one msg in buffer, cntd");
                        let packet: Packet = match bincode::deserialize(&buffer) {
                            Err(e) => {
                                error!("Received package is broken: {e}");
                                continue;
                            }
                            Ok(packet) => packet,
                        };

                        match channel.send((packet, partner.clone())) {
                            Err(e) => {
                                error!("TCP Listener could not send to internal channel: {e}");
                            }
                            _ => {}
                        };
                        buffer.clear();
                        is_for_us = false;
                        first = true;
                        msg_size = 0;
                    }
                }

                // the entire buf was read into correctly sized chunks for packets
                if !is_for_us || buffer.is_empty() {
                    debug!("not for us or buffer empty");
                    break;
                }

                // last packet was not read completely into the buf size so rerun the read command
            }
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
        self.coordinator_receive.write().unwrap().take();
        self.coordinator_send.write().unwrap().take();

        let network_register_packet = Packet {
            header: crate::net::Header {
                source: ShipName::MAX,
                target: CONTROLLER_CLIENT_ID,
            },
            data: PacketKind::JoinRequest(
                self.tcp_port,
                self.other_client_entrance,
                Sea::pad_ship_kind_name(&self.kind),
            ),
        };

        debug!(
            "register with client_entrance port: {}",
            self.other_client_entrance
        );

        let udp_socket = Self::get_udp_socket(Some(self.ip), None).await;

        loop {
            Self::send_packet_broadcast(&network_register_packet, &udp_socket).await?;

            let tim = tokio::time::timeout(CLIENT_REGISTER_TIMEOUT, self.connected.recv()).await;

            match tim {
                Err(_) => {
                    info!("JoinRequest timed out..");
                    continue;
                }
                Ok(None) => {
                    return Err(anyhow!("TCP channel closed"));
                }
                Ok(Some(_)) => {
                    break;
                }
            }
        }

        let (_disconnect_tx, disconnect_rx) = tokio::sync::oneshot::channel();

        // only make non lh nodes wait for the coordinate since they ask for variables
        if match &self.kind {
            ShipKind::Rat(name) => name != COMPARE_NODE_NAME,
            _ => true,
        } {
            // TODO disconnect not implemented yet, must send to disconnect_tx

            info!("waiting for coord ready signal.");
            // Wait for coordinator to send us Ack to signal that every expected client is connected
            let mut coord_sub = self
                .coordinator_receive
                .read()
                .unwrap()
                .as_ref()
                .expect("TCP Socket does not exist")
                .subscribe();
            loop {
                match coord_sub.recv().await {
                    Err(e) => {
                        error!("Could not receive updates from coordinator: {}", e);
                    }
                    Ok((packet, _)) => {
                        if matches!(packet.data, PacketKind::Acknowledge) {
                            debug!("received ack, so all are connected!");
                            break;
                        }
                    }
                }
            }
        }

        Ok(disconnect_rx)
    }

    pub async fn send_raw_to_other_client(
        &self,
        address: &IpAddr,
        port: u16,
        data: Vec<u8>,
        variable_type: VariableType,
    ) -> anyhow::Result<()> {
        loop {
            let addr = format!("{}:{}", address.to_string(), port);
            let stream = tokio::time::timeout(
                CLIENT_TO_CLIENT_TIMEOUT,
                tokio::net::TcpStream::connect(&addr),
            )
            .await;

            match stream {
                Err(_) => {
                    return Err(anyhow!(
                        "Could not connect to other client within {CLIENT_TO_CLIENT_TIMEOUT:?}"
                    ));
                }
                Ok(Err(e)) => match e.kind() {
                    std::io::ErrorKind::ConnectionRefused => {
                        // debug!("connection not yet ready, wait a bit");
                        tokio::task::yield_now().await;
                        continue;
                    }
                    _ => {
                        return Err(anyhow!("Could not connect to other client: {e}"));
                    }
                },
                Ok(Ok(mut stream)) => {
                    // ask if ready to receive by sending ack and wait for ack answer
                    let ack_data = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::Acknowledge,
                    };
                    let ack_data = bincode::serialize(&ack_data).expect("non serializable ack");
                    debug!("connected to {}", addr);
                    stream.writable().await?;

                    // ask for other client to get ready
                    {
                        let mut buf = vec![0; ack_data.len()];
                        loop {
                            stream.write_all(&ack_data).await?;
                            debug!("written ack_data, wait for response");

                            let timeout = tokio::time::timeout(
                                CLIENT_TO_CLIENT_INIT_RETRY_TIMEOUT,
                                stream.read_exact(&mut buf),
                            )
                            .await;

                            match timeout {
                                Ok(Ok(_)) => {
                                    let received_packet =
                                        bincode::deserialize::<crate::net::Packet>(&buf)?;
                                    if matches!(
                                        received_packet.data,
                                        crate::net::PacketKind::Acknowledge
                                    ) {
                                        break;
                                    }
                                    warn!("Did not receive ack packet");
                                }
                                Err(_) => {
                                    debug!(
                                        "Timeout receiving ack to start sending from other client."
                                    );
                                }
                                Ok(Err(e)) => {
                                    error!("Could not read from socket: {e}");
                                }
                            }
                        }
                    }

                    debug!("received ack from other client, sending now");
                    let packet_size = data.len() as u32;
                    let packet_size_buffer = packet_size.to_be_bytes();
                    let mut buffer = vec![
                        PROTO_IDENTIFIER,
                        packet_size_buffer[0],
                        packet_size_buffer[1],
                        packet_size_buffer[2],
                        packet_size_buffer[3],
                        variable_type.into(),
                    ];
                    buffer.extend_from_slice(&data);
                    stream.write_all(&buffer).await?;
                    debug!("written all");
                    return Ok(());
                }
            }
        }
    }

    pub async fn recv_raw_from_other_client(
        &self,
        sender: Option<&crate::NetworkShipAddress>,
    ) -> anyhow::Result<(Vec<u8>, VariableType)> {
        loop {
            let stream = tokio::time::timeout(
                CLIENT_TO_CLIENT_TIMEOUT,
                self.other_client_listener.accept(),
            )
            .await;

            debug!("recv from other");
            let mut variable_type = VariableType::default();
            match stream {
                Err(_) => {
                    return Err(anyhow!(
                        "Could not connect to other client within {CLIENT_TO_CLIENT_TIMEOUT:?}"
                    ));
                }
                Ok(Err(e)) => match e.kind() {
                    std::io::ErrorKind::ConnectionRefused => {
                        debug!("connection not yet ready, wait a bit");
                        tokio::task::yield_now().await;
                        continue;
                    }
                    _ => {
                        return Err(anyhow!("Could not connect to other client: {e}"));
                    }
                },
                Ok(Ok((mut stream, socket_sender))) => {
                    debug!("started receiving from {:?}", sender);
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

                    // Ask for other client if it is ready. Normally you would assume TCP handles handshakes etc but it seems to write
                    // to the socket into the void without the receiver being ready to receive anything. Tokio then merges the buffers when calling recv()?
                    // Another option would be to init the socket on each receive but then the socket could be set in the meantime by the operating system.
                    let ack_data = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::Acknowledge,
                    };
                    let ack_data = bincode::serialize(&ack_data).expect("non serializable ack");
                    let mut buf = vec![0; ack_data.len()];
                    stream.read_exact(&mut buf).await.map_err(|e| {
                        anyhow!("Could not read exact in receiver for other client comm: {e}")
                    })?;
                    let received_packet = bincode::deserialize::<crate::net::Packet>(&buf)?;
                    if !matches!(received_packet.data, crate::net::PacketKind::Acknowledge) {
                        return Err(anyhow!("Received packet for 1-1 handshake is not Ack"));
                    }

                    stream.write_all(&ack_data).await?;
                    debug!("handshake sucess");

                    let mut buffer = Vec::with_capacity(1024);
                    let mut first = true;
                    let mut msg_size = 0;

                    loop {
                        // Hint: This process only works as long as no other client is writing to our current stream. With the coordinator socket we sometimes get 2 messages into the same stream but they are from different clients talking to the same socket.
                        let mut buf = [0; 1024];
                        let n = match stream.read(&mut buf).await {
                            Err(e) => {
                                return Err(anyhow!("Could not read from TCP stream when receiving from other client: {e}"));
                            }
                            Ok(n) => n,
                        };

                        if n == 0 {
                            return Err(anyhow!(
                                "Stream seems to be broken while expecting the data."
                            ));
                        }

                        if first {
                            if buf[0] != PROTO_IDENTIFIER {
                                return Err(anyhow!("Other client sent unexpected start token."));
                            } else {
                                // read 4 bytes into u32 for length of rest message
                                let msg_size_buf = [buf[1], buf[2], buf[3], buf[4]];
                                variable_type = VariableType::from(buf[5]);
                                msg_size = u32::from_be_bytes(msg_size_buf);
                                first = false;
                            }
                        }

                        buffer.extend_from_slice(&buf[..n]);

                        if buffer.len() as u32 == msg_size + 6 {
                            break;
                        }
                    }

                    if !buffer.is_empty() {
                        buffer = buffer[6..buffer.len()].into(); // TODO for large data, maybe better to directly delete the indicators when checking them to avoid large copies
                        return Ok((buffer, variable_type));
                    }

                    return Err(anyhow!("Received data is not for us"));
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
        data.extend_from_slice(&raw);
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
