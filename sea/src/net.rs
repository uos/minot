use std::{
    io::Read,
    net::{IpAddr, TcpStream},
};

use anyhow::{anyhow, Error};
use log::{error, info};
use rand::random;
use tokio::{
    io::AsyncReadExt,
    net::{TcpListener, TcpSocket, UdpSocket},
};

use serde::{Deserialize, Serialize};

use pnet::{datalink, packet::PacketData};

use crate::ShipName;

const PROTO_IDENTIFIER: u8 = 69;
const CONTROLLER_CLIENT_ID: ShipName = 0;
const CLIENT_REGISTER_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(150);
const CLIENT_LISTEN_PORT: u16 = 6969;
const BI_CLIENT_LISTEN_PORT: u16 = 6970;

#[derive(Serialize, Deserialize, Clone)]
enum PacketKind {
    Acknowledge, // basically a Ping
    JoinRequest(u16),
    RawData(Vec<u8>),
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

#[derive(Serialize, Deserialize, Copy, Clone)]
struct Header {
    pub source: ShipName,
    pub target: ShipName,
}

#[derive(Serialize, Deserialize, Clone)]
struct Packet {
    header: Header,
    data: PacketKind,
}

struct Client {
    udp_socket: UdpSocket,
    tcp_listen_task: tokio::task::JoinHandle<()>,
    tcp_listener_chan: tokio::sync::mpsc::Receiver<(Packet, std::net::SocketAddr)>,
    udp_port: u16,
    tcp_port: u16,
    id: ShipName,
    ip: [u8; 4],
    interfaces: Vec<pnet::datalink::NetworkInterface>,
    in_network: bool,
}

impl Client {
    pub async fn new(controller: bool, external_ip: Option<[u8; 4]>) -> Self {
        let ip = external_ip.unwrap_or([0, 0, 0, 0]);
        let bind_address = format!("{}.{}.{}.{}:0", ip[0], ip[1], ip[2], ip[3]);
        let socket = std::net::UdpSocket::bind(&bind_address).unwrap();
        let udp_port = socket
            .local_addr()
            .expect("could not get sock adress")
            .port();
        socket.set_broadcast(true).expect("could not set broadcast");
        let udp_socket = UdpSocket::from_std(socket).expect("could not promote to tokio socket");
        udp_socket
            .writable()
            .await
            .expect("udp socket not writable");

        let tcp_port = if controller { CLIENT_LISTEN_PORT } else { 0 };
        let bind_address = format!("{}.{}.{}.{}:{}", ip[0], ip[1], ip[2], ip[3], tcp_port);
        let socket = std::net::TcpListener::bind(&bind_address).expect("could not bind tcp socket");
        let tcp_port = socket
            .local_addr()
            .expect("could not get tcp listener adress")
            .port();
        let tcp_listener =
            TcpListener::from_std(socket).expect("could not create tokio tcp listener");

        let tmp_id = if controller {
            CONTROLLER_CLIENT_ID
        } else {
            let mut random: i128 = 0;
            while random == 0 {
                random = rand::random();
            }
            if random > 0 {
                random * -1
            } else {
                random
            }
        };

        let interfaces = datalink::interfaces()
            .into_iter()
            .filter(|i| i.is_running())
            .collect::<Vec<_>>(); // TODO maybe !i.is_loopback()?

        let (tx, rx) = tokio::sync::mpsc::channel(100);
        let listen_task = tokio::spawn(async move {
            loop {
                match tcp_listener.accept().await {
                    Err(e) => {
                        error!("failed to accept tcp connection: {}", e);
                    }

                    Ok((mut stream, partner)) => {
                        info!("Established connection with {}", partner);
                        match stream.readable().await {
                            Err(e) => {
                                error!("TCP Listener stream not readable: {}", e);
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
                                    error!("Could not read from TCP stream: {}", e);
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

                        match tx.send((packet, partner)).await {
                            Err(e) => {
                                error!("TCP Listener could not send to internal channel: {}", e);
                            }
                            _ => {}
                        };
                    }
                }
            }
        });

        Self {
            interfaces,
            tcp_listener_chan: rx,
            tcp_listen_task: listen_task,
            udp_socket,
            tcp_port,
            udp_port,
            id: tmp_id,
            in_network: false,
            ip,
        }
    }

    // TODO need to wait for ctrl-c to handle this with grace and async
    pub async fn dissolve_network(clients: &[Client]) {
        // TODO tell all our clients to gtfo. They need to reschedule their request to get back into a network again.
    }

    pub async fn register(&mut self) -> anyhow::Result<()> {
        let network_register_packet = Packet {
            header: Header {
                source: self.id,
                target: CONTROLLER_CLIENT_ID,
            },
            data: PacketKind::JoinRequest(self.tcp_port),
        };
        self.send_packet_broadcast(&network_register_packet).await?;

        let tim =
            tokio::time::timeout(CLIENT_REGISTER_TIMEOUT, self.tcp_listener_chan.recv()).await;

        match tim {
            Err(_) => {
                return Err(anyhow!("JoinRequest timed out"));
            }
            Ok(None) => {
                return Err(anyhow!("TCP Channel closed"));
            }
            Ok(Some((packet, _partner))) => {
                if !matches!(packet.data, PacketKind::Acknowledge) {
                    return Err(anyhow!("JoinRequest denied :("));
                }
                self.in_network = true; // TODO handle rescheduled reconnect tries after our network was cancelled
                return Ok(());
            }
        };
    }

    fn calculate_broadcast(ipv4_addr: std::net::Ipv4Addr, prefix_len: u8) -> std::net::Ipv4Addr {
        let mask = !0u32 >> prefix_len;
        let bcast_addr = u32::from(ipv4_addr) | mask;
        std::net::Ipv4Addr::from(bcast_addr)
    }

    async fn send_packet_broadcast(&self, packet: &Packet) -> anyhow::Result<()> {
        let raw = bincode::serialize(packet).expect("packet not serializable");
        let mut data = vec![PROTO_IDENTIFIER];
        data.extend(raw);

        for interface in self.interfaces.iter() {
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

                    let sent = self.udp_socket.send_to(&data, &target_str).await?;
                    if sent != data.len() {
                        return Err(anyhow!("data len mismatch with actual send bytes"));
                    }
                }
            }
        }

        Ok(())
    }
}

struct Sea {
    coordinator_client: Client,
    network_clients: Vec<Client>,
}

impl Sea {
    pub async fn new(external_ip: Option<[u8; 4]>) -> Self {
        Self {
            coordinator_client: Client::new(true, external_ip).await,
            network_clients: Vec::new(),
        }
    }
}
