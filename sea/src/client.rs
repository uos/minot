use std::{
    collections::HashMap,
    net::{IpAddr, SocketAddr, TcpStream},
    time::Duration,
};

use anyhow::anyhow;
use deadpool::managed::{Manager, Object, RecycleError};
use log::{debug, error, info};
use net::COMPARE_NODE_NAME;
use pnet::datalink::{self, NetworkInterface};
use rkyv::{api::low::from_bytes, to_bytes, util::AlignedVec};
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
        CONTROLLER_CLIENT_ID, PROTO_IDENTIFIER, Packet, PacketKind, Sea,
    },
};
const COMM_HEADER_BYTES_N: usize = 1 + std::mem::size_of::<u32>();

#[derive(Debug)]
pub struct TcpManager {
    pub addr: String,
}

impl Manager for TcpManager {
    type Type = tokio::net::TcpStream;
    type Error = anyhow::Error;

    fn create(&self) -> impl Future<Output = Result<Self::Type, Self::Error>> + Send {
        let addr = self.addr.clone();
        Box::pin(async move {
            let stream = tokio::net::TcpStream::connect(addr.clone())
                .await
                .map_err(|e| anyhow!("Failed to connect to {}: {}", addr, e));
            stream
        })
    }

    /// Corrected recycle logic.
    /// We simply assume the connection is healthy. If it's not, the next
    /// operation on it will fail, and deadpool will discard it correctly.
    fn recycle(
        &self,
        _conn: &mut Self::Type,
        _metrics: &deadpool::managed::Metrics,
    ) -> impl Future<Output = Result<(), RecycleError<Self::Error>>> + Send {
        Box::pin(async { Ok(()) })
    }
}

type TcpPool = deadpool::managed::Pool<TcpManager>;

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
    pub other_client_entrance: u16, // tcp port for 1:1 connections
    pub kind: ShipKind,
    connected: tokio::sync::mpsc::Receiver<bool>,
    rm_rules_on_disconnect: bool,
    pub updated_raw_recv: tokio::sync::broadcast::Sender<u32>,
    pub raw_recv_buff:
        std::sync::Arc<std::sync::RwLock<HashMap<u32, Vec<(Vec<u8>, VariableType, String)>>>>,
    pools: std::sync::Arc<tokio::sync::Mutex<HashMap<String, TcpPool>>>,
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
        let udp_socket = match UdpSocket::bind(&bind_address).await {
            Ok(s) => s,
            Err(e) => {
                if e.kind() == std::io::ErrorKind::AddrInUse {
                    if let Some(_) = port {
                        error!(
                            "Another Minot coordinator instance is likely running. Terminate the other instance before starting a new one."
                        );
                    } else {
                        error!(
                            "Dynamic UDP port already in use ({}). Terminate the other instance if it is still running.",
                            bind_address
                        );
                    }
                    std::process::exit(1);
                } else {
                    error!("Failed to bind UDP socket at {}: {}", bind_address, e);
                    std::process::exit(1);
                }
            }
        };
        udp_socket
            .set_broadcast(true)
            .expect("could not set broadcast");
        udp_socket
            .writable()
            .await
            .expect("udp socket not writable");
        udp_socket
    }

    async fn get_pool(&self, addr: &String) -> TcpPool {
        let mut pools = self.pools.lock().await;
        if let Some(pool) = pools.get(addr) {
            return pool.clone();
        }
        let manager = TcpManager { addr: addr.clone() };
        let pool = deadpool::managed::Pool::builder(manager)
            .max_size(1)
            .build()
            .unwrap();
        pools.insert(addr.clone(), pool.clone());
        pool
    }

    pub async fn init(
        ship_kind: ShipKind,
        external_ip: Option<[u8; 4]>,
        rm_rules_on_disconnect: bool,
    ) -> Self {
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
        let (updated_raw_recv, _) = tokio::sync::broadcast::channel(100);
        let raw_recv_buff = std::sync::Arc::new(std::sync::RwLock::new(HashMap::new()));

        let inner_updated_raw_recv = updated_raw_recv.clone();
        let inner_raw_recv_buff = std::sync::Arc::clone(&raw_recv_buff);

        // receiving packets from other clients
        tokio::spawn(async move {
            Self::recv_raw_from_other_client(
                client_listener,
                inner_updated_raw_recv,
                inner_raw_recv_buff,
            )
            .await;
        });

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
                        if let Err(e) = stream.readable().await {
                            error!("TCP Listener stream not readable: {e}");
                            continue;
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
                            if let PacketKind::Welcome { addr, wait_for_ack } = packet.data {
                                // TODO maybe send the coord_send_tx_inner directly if someone wants to use it
                                connected_tx_inner.send(wait_for_ack).await.unwrap();
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
                                            packet.header.source = addr.ship; // overwrite source with our id
                                            packet.header.target = CONTROLLER_CLIENT_ID; // overwrite target with controller id
                                            if let Err(e) = coord_raw_sender.send(packet).await {
                                                error!("could not forward coordinator packet: {e}");
                                            }
                                        }
                                        Err(_e) => {
                                            let packet = Packet {
                                                header: crate::net::Header {
                                                    source: addr.ship,
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
            coordinator_send: coord_raw_send_tx,
            coordinator_receive: coord_raw_receive_tx,
            connected: connected_rx,
            tcp_port,
            ip,
            rm_rules_on_disconnect,
            updated_raw_recv,
            raw_recv_buff,
            pools: std::sync::Arc::new(tokio::sync::Mutex::new(HashMap::new())),
        }
    }

    pub async fn send_to_socket(
        mut channel: tokio::sync::mpsc::Receiver<Packet>,
        mut wh: OwnedWriteHalf,
    ) {
        while let Some(packet) = channel.recv().await {
            let packet_bytes = match to_bytes::<rkyv::rancor::Error>(&packet) {
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
            debug!("Sent buffer through socket");
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
                    debug!("TCP connection closed by peer.");
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

                    let packet: Packet = match from_bytes::<Packet, rkyv::rancor::Error>(
                        &packet_bytes,
                    ) {
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

                    if let Err(e) = channel.send((packet, partner)) {
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
            data: PacketKind::JoinRequest {
                tcp_port: self.tcp_port,
                other_client_entrance: self.other_client_entrance,
                kind: Sea::pad_ship_kind_name(&self.kind),
                remove_rules_on_disconnect: self.rm_rules_on_disconnect,
            },
        };

        debug!(
            "register with client_entrance port: {}",
            self.other_client_entrance
        );

        let udp_socket = Self::get_udp_socket(Some(self.ip), None).await;

        let wait_for_ack = loop {
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
                Ok(Some(wait_for_ack)) => {
                    break wait_for_ack;
                }
            }
        };

        let (_disconnect_tx, disconnect_rx) = tokio::sync::oneshot::channel();

        // only make non minot tui nodes wait for the coordinate since they ask for variables
        if match &self.kind {
            ShipKind::Rat(name) => name != COMPARE_NODE_NAME,
            _ => true,
        } {
            // TODO disconnect not implemented yet, must send to disconnect_tx
            if wait_for_ack {
                info!("{:?}: waiting for coord ready signal.", self.kind);
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
        }

        Ok(disconnect_rx)
    }

    pub async fn send_raw_to_other_client(
        &self,
        address: &IpAddr,
        id: u32,
        port: u16,
        data: AlignedVec,
        variable_type: VariableType,
        variable_name: &str,
    ) -> anyhow::Result<()> {
        let addr = format!("{}:{}", address, port);
        let pool = self.get_pool(&addr).await;

        // A small retry loop for getting a connection or handling a write error
        const MAX_RETRIES: u32 = 2;
        for attempt in 0..MAX_RETRIES {
            let mut stream = match pool.get().await {
                Ok(s) => s,
                Err(e) => {
                    error!(
                        "Attempt {}: Failed to get connection from pool: {}",
                        attempt + 1,
                        e
                    );
                    tokio::time::sleep(Duration::from_millis(50)).await;
                    continue;
                }
            };

            // --- Frame the message once ---
            let mut padded_name = [0u8; 64];
            let name_bytes = variable_name.as_bytes();
            let len = name_bytes.len().min(64);
            padded_name[..len].copy_from_slice(&name_bytes[..len]);

            let packet_size = data.len() as u32;
            let packet_size_buffer = packet_size.to_be_bytes();
            let id_buffer = id.to_be_bytes();

            let mut header = vec![
                PROTO_IDENTIFIER,      // 1 byte
                packet_size_buffer[0], // 4 bytes for size
                packet_size_buffer[1],
                packet_size_buffer[2],
                packet_size_buffer[3],
                variable_type.into(), // 1 byte
                id_buffer[0],         // 4 bytes for ID
                id_buffer[1],
                id_buffer[2],
                id_buffer[3],
            ];
            header.extend_from_slice(&padded_name); // 64 bytes for name

            match stream.write_all(&header).await {
                Ok(_) => (),
                Err(e) => {
                    error!(
                        "Attempt {}: Failed to write header: {}. Invalidating connection.",
                        attempt + 1,
                        e
                    );
                    let _ = Object::take(stream); // Purge bad connection
                    continue; // Retry
                }
            }

            match stream.write_all(&data).await {
                Ok(_) => {
                    debug!("Successfully wrote data for id {} to stream.", id);
                    return Ok(());
                }
                Err(e) => {
                    error!(
                        "Attempt {}: Failed to write data: {}. Invalidating connection.",
                        attempt + 1,
                        e
                    );
                    let _ = Object::take(stream); // Purge bad connection
                    continue; // Retry
                }
            }
        }

        Err(anyhow!("Failed to send data after {} retries", MAX_RETRIES))
    }

    /// A task to distribute received messages from other clients via 1 to 1 TCP connections
    pub async fn recv_raw_from_other_client(
        tcp_port: tokio::net::TcpListener,
        updated_raw_recv: tokio::sync::broadcast::Sender<u32>,
        raw_recv_buff: std::sync::Arc<
            std::sync::RwLock<HashMap<u32, Vec<(Vec<u8>, VariableType, String)>>>,
        >,
    ) -> ! {
        const HEADER_SIZE: usize = 1 + 4 + 1 + 4 + 64; // PROTO(1) + SIZE(4) + TYPE(1) + ID(4) + NAME(64)
        loop {
            let (mut stream, addr) = match tcp_port.accept().await {
                Ok(res) => res,
                Err(e) => {
                    error!("Failed to accept connection: {e}");
                    continue;
                }
            };
            debug!("Accepted new connection from {}", addr);

            let task_update_chan = updated_raw_recv.clone();
            let task_recv_buff = std::sync::Arc::clone(&raw_recv_buff);

            tokio::spawn(async move {
                let mut header_buf = [0u8; HEADER_SIZE];

                loop {
                    match stream.read_exact(&mut header_buf).await {
                        Ok(_) => (),
                        Err(e) => {
                            // "early eof" is normal when the client disconnects cleanly.
                            if e.kind() != std::io::ErrorKind::UnexpectedEof {
                                error!(
                                    "Could not read message header from {}: {}. Closing connection.",
                                    addr, e
                                );
                            } else {
                                debug!("Connection closed by {}.", addr);
                            }
                            break;
                        }
                    };

                    if header_buf[0] != PROTO_IDENTIFIER {
                        error!(
                            "Invalid protocol identifier from {}. Closing connection.",
                            addr
                        );
                        break;
                    }

                    let size_bytes = [header_buf[1], header_buf[2], header_buf[3], header_buf[4]];
                    let msg_size = u32::from_be_bytes(size_bytes);

                    let variable_type = VariableType::from(header_buf[5]);

                    let id_bytes = [header_buf[6], header_buf[7], header_buf[8], header_buf[9]];
                    let msg_id = u32::from_be_bytes(id_bytes);

                    let name_bytes = &header_buf[10..];
                    let var_name = String::from_utf8_lossy(
                        name_bytes.split(|&b| b == 0).next().unwrap_or_default(),
                    )
                    .to_string();

                    let mut payload_buf = vec![0; msg_size as usize];
                    if let Err(e) = stream.read_exact(&mut payload_buf).await {
                        error!(
                            "Failed to read payload of size {} for id {}: {}. Closing connection.",
                            msg_size, msg_id, e
                        );
                        break;
                    }

                    let response_data = (payload_buf, variable_type, var_name);

                    {
                        let mut lock = task_recv_buff.write().unwrap();
                        lock.entry(msg_id).or_default().push(response_data);
                    }

                    if task_update_chan.send(msg_id).is_err() {
                        debug!(
                            "Data for id {} is ready, but no consumers are listening.",
                            msg_id
                        );
                    }
                }
            });
        }
    }

    // --- end send/recv ---

    fn calculate_broadcast(ipv4_addr: std::net::Ipv4Addr, prefix_len: u8) -> std::net::Ipv4Addr {
        let mask = !0u32 >> prefix_len;
        let bcast_addr = u32::from(ipv4_addr) | mask;
        std::net::Ipv4Addr::from(bcast_addr)
    }

    async fn send_packet_broadcast(packet: &Packet, udp_socket: &UdpSocket) -> anyhow::Result<()> {
        let raw = to_bytes::<rkyv::rancor::Error>(packet).expect("packet not serializable");
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
