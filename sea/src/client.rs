use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream};

use anyhow::anyhow;
use log::{debug, error, info, warn};
use pnet::datalink::{self, NetworkInterface};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{
        UdpSocket,
        tcp::{OwnedReadHalf, OwnedWriteHalf},
    },
};

use crate::{
    ShipKind, ShipName, VariableType,
    net::{
        CLIENT_LISTEN_PORT, CLIENT_REGISTER_TIMEOUT, CLIENT_REJOIN_POLL_INTERVAL,
        CLIENT_TO_CLIENT_INIT_RETRY_TIMEOUT, CLIENT_TO_CLIENT_TIMEOUT, CONTROLLER_CLIENT_ID,
        Header, PROTO_IDENTIFIER, Packet, PacketKind, Sea,
    },
};
const COMM_HEADER_BYTES_N: usize = 1 + std::mem::size_of::<u32>();

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
        let udp_socket = UdpSocket::bind(&bind_address).await.unwrap();
        udp_socket
            .set_broadcast(true)
            .expect("could not set broadcast");
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
        let tcp_listener = tokio::net::TcpListener::bind(&bind_address)
            .await
            .expect("could not bind tcp socket");

        let tcp_port = tcp_listener
            .local_addr()
            .expect("could not get tcp listener adress")
            .port();

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
                                                    error!(
                                                        "could not send heartbeat to coordinator: {e}"
                                                    );
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
            let packet_bytes = match packet.to_bytes() {
                Ok(bytes) => bytes,
                Err(e) => {
                    error!("Failed to serialize packet: {e}");
                    // Log the error and skip sending this specific packet.
                    // A serialization error doesn't necessarily mean the connection is broken.
                    continue;
                }
            };

            let packet_size = packet_bytes.len();

            // Check if the packet size fits into a u32
            if packet_size > u32::MAX as usize {
                error!(
                    "Packet size ({}) exceeds maximum allowed {} bytes for size header. Dropping packet.",
                    packet_size,
                    u32::MAX
                );
                continue; // Cannot send packets this large with this protocol
            }

            let packet_size_u32 = packet_size as u32;
            let packet_size_buffer = packet_size_u32.to_be_bytes();
            let mut buffer = Vec::with_capacity(COMM_HEADER_BYTES_N + packet_size);
            buffer.push(PROTO_IDENTIFIER);
            buffer.extend_from_slice(&packet_size_buffer);
            buffer.extend_from_slice(&packet_bytes);

            debug!(
                "Attempting to send {} bytes (Packet size: {}).",
                buffer.len(),
                packet_size
            );

            if let Err(e) = wh.write_all(&buffer).await {
                error!("Failed to send data to TCP socket: {e}");
                channel.close();
                return;
            }
        }
        info!("Send channel closed, send_to_socket task exiting.");
    }

    pub async fn receive_from_socket(
        mut rh: OwnedReadHalf,
        channel: tokio::sync::broadcast::Sender<(Packet, Option<SocketAddr>)>,
        partner: Option<SocketAddr>,
    ) {
        let mut received_data: Vec<u8> = Vec::new();
        let mut temp_buffer = [0; 4096];

        loop {
            let n = match rh.read(&mut temp_buffer).await {
                Ok(0) => {
                    // Connection closed by peer
                    info!("TCP connection closed by peer.");
                    return;
                }
                Ok(n) => n,
                Err(e) => {
                    error!("Failed to read from TCP stream: {}", e);
                    return;
                }
            };

            debug!("Read {} bytes from socket for partner {:?}.", n, partner);

            received_data.extend_from_slice(&temp_buffer[..n]);

            // --- Packet Framing and Processing ---
            // Process the accumulated data buffer. We loop because a single read() might
            // contain multiple complete packets or the end of one packet and the start of another.
            while received_data.len() >= COMM_HEADER_BYTES_N {
                let identifier = received_data[0];
                if identifier != PROTO_IDENTIFIER {
                    error!(
                        "Protocol identifier mismatch. Expected {}, got {}. Disconnecting.",
                        PROTO_IDENTIFIER, identifier
                    );
                    // TODO maybe just drop the packet
                    return;
                }

                let size_bytes: [u8; 4] = match received_data[1..COMM_HEADER_BYTES_N].try_into() {
                    Ok(bytes) => bytes,
                    Err(_) => {
                        error!(
                            "Buffer too short to extract size bytes. Length: {}. Expected at least {}. Disconnecting.",
                            received_data.len(),
                            COMM_HEADER_BYTES_N
                        );
                        return;
                    }
                };
                let msg_size = u32::from_be_bytes(size_bytes) as usize;

                debug!(
                    "Found header for packet of size {} for partner {:?}. Total needed: {}.",
                    msg_size,
                    partner,
                    COMM_HEADER_BYTES_N + msg_size
                );

                // Check if the accumulation buffer contains the complete packet (header + data)
                let total_packet_size = COMM_HEADER_BYTES_N + msg_size;
                if received_data.len() >= total_packet_size {
                    // We have enough data for a complete packet!

                    // Extract the packet data (excluding the header)
                    let packet_bytes =
                        received_data[COMM_HEADER_BYTES_N..total_packet_size].to_vec();
                    debug!(
                        "Extracted packet bytes ({} bytes) for partner {:?}.",
                        packet_bytes.len(),
                        partner
                    );

                    let packet: Packet = match bincode::deserialize(&packet_bytes) {
                        Ok(packet) => packet,
                        Err(e) => {
                            error!(
                                "Failed to deserialize packet for partner {:?}: {}. Discarding this packet.",
                                partner, e
                            );
                            // Discard the bytes for the corrupted packet from the buffer
                            received_data.drain(..total_packet_size);
                            // Continue processing the *rest* of the buffer
                            continue;
                        }
                    };

                    debug!("Packet details: {:?}", packet);

                    if let Err(e) = channel.send((packet, partner.clone())) {
                        error!(
                            "Could not send packet to internal broadcast channel for partner {:?}: {}",
                            partner, e
                        );
                        // The channel is likely closed or has no active receivers.
                        return;
                    }
                    debug!(
                        "Sent packet to broadcast channel for partner {:?}.",
                        partner
                    );

                    // Remove the processed packet (header + data) from the beginning of the buffer.
                    received_data.drain(..total_packet_size);
                    debug!(
                        "Drained {} bytes from buffer. Remaining buffer size: {}.",
                        total_packet_size,
                        received_data.len(),
                    );

                    // Loop back to check if the next bytes in the buffer form another complete packet
                } else {
                    debug!(
                        "Buffer incomplete for next packet. Needed: {}, Available: {}. Waiting for more data.",
                        total_packet_size,
                        received_data.len()
                    );
                    break;
                }
            }

            // If the inner loop finished, we either processed all available complete packets
            // or the remaining data in the buffer is less than the size of a header, or
            // less than the size of the *next* expected packet.
            // The outer loop continues to wait for more data from the socket.
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
                    debug!("JoinRequest timed out..");
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
            ShipKind::Rat(name) => name != rlc::COMPARE_NODE_NAME,
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
        variable_name: &str,
    ) -> anyhow::Result<()> {
        loop {
            let addr = format!("{}:{}", address.to_string(), port);
            let stream = tokio::time::timeout(
                CLIENT_TO_CLIENT_TIMEOUT,
                tokio::net::TcpStream::connect(&addr),
            )
            .await;

            let padded_var_name = Sea::pad_string(variable_name);

            match stream {
                Err(_) => {
                    return Err(anyhow!(
                        "Could not connect to other client within {CLIENT_TO_CLIENT_TIMEOUT:?}"
                    ));
                }
                Ok(Err(e)) => match e.kind() {
                    std::io::ErrorKind::ConnectionRefused => {
                        tokio::task::yield_now().await;
                        continue;
                    }
                    _ => {
                        return Err(anyhow!("Could not connect to other client: {e}"));
                    }
                },
                Ok(Ok(mut stream)) => {
                    // ask if ready to receive by sending ack and wait for ack answer
                    let init_request = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::RequestVarSend(padded_var_name),
                    };
                    let init_request_data =
                        bincode::serialize(&init_request).expect("non serializable ack");
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
                            stream.write_all(&init_request_data).await?;
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
                                    warn!(
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

                    let expected_retry = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::Retry,
                    };
                    let retry_data =
                        bincode::serialize(&expected_retry).expect("non serializable retry");
                    let mut recv_buf = vec![0; retry_data.len()];
                    let mut send_buf = [0; 1024];
                    let mut send_cursor = 0;

                    let (mut stream_rx, mut stream_tx) = stream.split();
                    loop {
                        let mut sent_all = false;
                        let mut last_i = 0;
                        for i in 0..send_buf.len() {
                            if send_cursor >= buffer.len() {
                                sent_all = true;
                                break;
                            }
                            last_i = i;
                            send_buf[i] = buffer[send_cursor];
                            send_cursor += 1;
                        }

                        if send_cursor == buffer.len() {
                            sent_all = true;
                        }

                        // truncating buffer to not send trash bytes
                        let send_buf = if sent_all && last_i + 1 < send_buf.len() {
                            &send_buf[0..last_i + 1]
                        } else {
                            &send_buf
                        };
                        tokio::select! {
                            fd_recv = stream_rx.read_exact(&mut recv_buf) => {
                                let n = fd_recv.expect("Could not read from tcp stream.");
                                if n == 0 {
                                    debug!("read empty stream while sending to other client.");
                                    continue;
                                }
                                    let received_packet =
                                        bincode::deserialize::<crate::net::Packet>(&recv_buf)?;
                                    if matches!(
                                        received_packet.data,
                                        crate::net::PacketKind::Retry
                                    ) {
                                        send_cursor = 0;
                                        continue;
                                    }
                                    warn!("Received something else than retry packet mid-stream.");
                            },
                            fd_sent = stream_tx.write(&send_buf) => {
                                let n = fd_sent.expect("Could not write to tcp stream.");
                                if n == 0 {
                                    error!("written 0 bytes to other client?");
                                    break;
                                }
                                debug!("written");
                            }
                        };
                        if sent_all {
                            stream_tx.flush().await?;
                            break;
                        }
                    }

                    // TODO in the current impl, the last packet can not be retried. maybe we could wait for an ack or retry here in loop and wait until we get one
                    // TODO maybe change every stream for rule conversation etc. to use this communication pattern

                    // stream.write_all(&buffer).await?;
                    debug!("written all");
                    return Ok(());
                }
            }
        }
    }

    pub async fn recv_raw_from_other_client(
        &self,
        sender: Option<&crate::NetworkShipAddress>,
    ) -> anyhow::Result<(Vec<u8>, VariableType, String)> {
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
                    let handshake_init_data = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::RequestVarSend(Sea::pad_string(" ")),
                    };
                    let handshake_init_data =
                        bincode::serialize(&handshake_init_data).expect("non serializable ack");
                    let mut buf = vec![0; handshake_init_data.len()];
                    stream.read_exact(&mut buf).await.map_err(|e| {
                        anyhow!("Could not read exact in receiver for other client comm: {e}")
                    })?;
                    let received_packet = bincode::deserialize::<crate::net::Packet>(&buf)?;
                    let var_name = if let crate::net::PacketKind::RequestVarSend(var_name) =
                        received_packet.data
                    {
                        var_name
                    } else {
                        return Err(anyhow!(
                            "Received packet for 1-1 handshake is of unexpected type"
                        ));
                    };
                    let var_name = Sea::reverse_padding(&var_name);

                    let handshake_start_ack = crate::net::Packet {
                        header: Header::default(),
                        data: PacketKind::Acknowledge,
                    };
                    let handshake_start_ack_data =
                        bincode::serialize(&handshake_start_ack).expect("non serializable ack");

                    stream.write_all(&handshake_start_ack_data).await?;
                    debug!("handshake sucess {}", var_name);

                    let mut buffer = Vec::with_capacity(1024);
                    let mut first = true;
                    let mut msg_size = 0;

                    loop {
                        // Hint: This process only works as long as no other client is writing to our current stream. With the coordinator socket we sometimes get 2 messages into the same stream but they are from different clients talking to the same socket.
                        let mut buf = [0; 1024];
                        let n = match stream.read(&mut buf).await {
                            Err(e) => {
                                return Err(anyhow!(
                                    "Could not read from TCP stream when receiving from other client: {e}"
                                ));
                            }
                            Ok(n) => n,
                        };

                        if n == 0 {
                            continue;
                        }

                        if first {
                            if buf[0] != PROTO_IDENTIFIER {
                                let retry_packet = crate::net::Packet {
                                    header: Header::default(),
                                    data: PacketKind::Retry,
                                };
                                let retry_packet_data = bincode::serialize(&retry_packet)
                                    .expect("non serializable retry");

                                stream.write_all(&retry_packet_data).await?;
                                buffer.clear();
                                first = true;
                                // warn!("Unexpected start token, asking for retry");
                                continue;
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
                        return Ok((buffer, variable_type, var_name));
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
