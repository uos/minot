use std::collections::HashSet;

use log::{debug, error, info, warn};
use nalgebra::DMatrix;

use rkyv::{Archive, Deserialize, Serialize, api::high::from_bytes, rancor, util::AlignedVec};
use zenoh::Wait;

use crate::{Action, ShipKind, ShipName, VariableHuman, WindData};

pub const PROTO_IDENTIFIER: u8 = 69;
pub const CONTROLLER_CLIENT_ID: ShipName = 0;

/// Copy bytes into an aligned buffer for rkyv deserialization
fn align_bytes(bytes: &[u8]) -> AlignedVec {
    let mut aligned = AlignedVec::with_capacity(bytes.len());
    aligned.extend_from_slice(bytes);
    aligned
}

/// Sanitize a ship name for use in Zenoh key expressions.
/// Zenoh forbids `#` and `?` characters in key expressions.
pub fn sanitize_key(name: &str) -> String {
    name.replace('#', "_hash_").replace('?', "_qmark_")
}

pub fn get_domain_id() -> u16 {
    let val = std::env::var("MINOT_DOMAIN_ID")
        .ok()
        .unwrap_or("0".to_owned());
    let parsed = val.parse::<u16>().ok();
    match parsed {
        Some(parsed) => parsed,
        None => {
            warn!("Invalid MINOT_DOMAIN_ID, selecting default 0");
            0
        }
    }
}

#[derive(Archive, Serialize, Deserialize, Clone, Debug)]
pub struct WindAt {
    pub data: WindData,
    pub at_var: Option<String>,
}

/// A wrapper type for using 0.8 rkyv APIs with nalgebra
#[derive(Archive, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct NetArray<T: nalgebra::Scalar> {
    cols: usize,
    data: Vec<T>,
    rows: usize,
}

impl<T: nalgebra::Scalar> From<DMatrix<T>> for NetArray<T> {
    fn from(value: DMatrix<T>) -> Self {
        Self {
            rows: value.nrows(),
            cols: value.ncols(),
            data: value.data.into(),
        }
    }
}

impl<T: nalgebra::Scalar> From<NetArray<T>> for DMatrix<T> {
    fn from(value: NetArray<T>) -> Self {
        Self::from_data(nalgebra::VecStorage::new(
            nalgebra::Dyn(value.rows),
            nalgebra::Dyn(value.cols),
            value.data,
        ))
    }
}

#[derive(Serialize, Deserialize, Archive, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RatPubRegisterKind {
    Publish,
    Subscribe,
}

#[derive(Serialize, Deserialize, Archive, Clone, Debug)]
pub enum PacketKind {
    Acknowledge,
    Retry,
    RequestVarSend(String),
    JoinRequest {
        kind: ShipKind,
        remove_rules_on_disconnect: bool,
        domain_id: u16,
    },
    Welcome {
        addr: crate::NetworkShipAddress,
        wait_for_ack: bool,
    },
    Heartbeat,
    Disconnect,
    RuleAppend {
        variable: String,
        commands: Vec<VariableHuman>,
    },
    RulesClear,
    LockNext {
        unlock_first: bool,
    },
    Unlock,
    RawDataf64(NetArray<f64>),
    RawDataf32(NetArray<f32>),
    RawDatai32(NetArray<i32>),
    RawDatau8(NetArray<u8>),
    VariableTaskRequest(String),
    RatAction {
        variable: String,
        action: Action,
        lock_until_ack: bool,
    },
    Wind(Vec<WindAt>),
    WindDynamic(String),
    RegisterShipAtVar {
        ship: String,
        var: String,
        kind: RatPubRegisterKind,
    },
}

#[derive(Archive, Serialize, Deserialize, Copy, Clone, Debug, Default)]
pub struct Header {
    pub source: ShipName,
    pub target: ShipName,
}

#[derive(Archive, Serialize, Deserialize, Clone, Debug)]
pub struct Packet {
    pub header: Header,
    pub data: PacketKind,
}

#[derive(Clone, Debug)]
pub struct ShipHandle {
    pub name: ShipKind,
    pub addr_from_coord: crate::NetworkShipAddress,
    pub ship: ShipName,
    pub disconnect: tokio::sync::broadcast::Sender<bool>,
    pub recv: tokio::sync::broadcast::Sender<(Packet, Option<std::net::SocketAddr>)>,
    pub send: tokio::sync::mpsc::Sender<Packet>,
    pub remove_rules_on_disconnect: bool,
}

/// Zenoh-based Sea coordinator
#[derive(Debug)]
pub struct Sea {
    pub network_clients_chan: tokio::sync::broadcast::Sender<ShipHandle>,
    #[allow(dead_code)]
    session: std::sync::Arc<zenoh::Session>,
    #[allow(dead_code)]
    domain_id: u16,
}

impl Sea {
    pub async fn init(
        _external_ip: Option<[u8; 4]>,
        clients_wait_for_ack: std::sync::Arc<std::sync::RwLock<bool>>,
    ) -> Self {
        let domain_id = get_domain_id();
        if domain_id > 0 {
            info!("Coordinator using domain ID {}", domain_id);
        }

        // Initialize Zenoh session
        let config = zenoh::Config::default();
        let session = zenoh::open(config)
            .wait()
            .expect("Failed to open Zenoh session");
        let session = std::sync::Arc::new(session);

        let (clients_tx, _clients_rx) = tokio::sync::broadcast::channel::<ShipHandle>(64);
        let clients_tx_inner = clients_tx.clone();

        // Key expression for join requests
        let join_key = format!("minot/{}/coord/join", domain_id);

        // Subscribe to join requests from clients
        let session_clone = std::sync::Arc::clone(&session);
        let clwa = std::sync::Arc::clone(&clients_wait_for_ack);
        let rat_lock = std::sync::Arc::new(std::sync::Mutex::new(HashSet::new()));

        tokio::spawn(async move {
            let subscriber = session_clone
                .declare_subscriber(join_key.clone())
                .wait()
                .expect("Failed to create join subscriber");

            info!("Sea coordinator listening on {}", join_key);

            loop {
                let sample = subscriber.recv_async().await;
                let sample = match sample {
                    Ok(s) => s,
                    Err(e) => {
                        error!("Error receiving join request: {}", e);
                        continue;
                    }
                };

                let payload = sample.payload().to_bytes();
                let aligned = align_bytes(&payload);
                let packet: Packet = match from_bytes::<Packet, rancor::Error>(&aligned) {
                    Ok(p) => p,
                    Err(e) => {
                        error!("Failed to deserialize join request: {}", e);
                        continue;
                    }
                };

                if let PacketKind::JoinRequest {
                    kind: ship_kind,
                    remove_rules_on_disconnect,
                    domain_id: client_domain_id,
                } = packet.data
                {
                    if client_domain_id != domain_id {
                        debug!(
                            "Rejecting join request from domain {} (coordinator is domain {})",
                            client_domain_id, domain_id
                        );
                        continue;
                    }

                    let ship_kind = Sea::unpad_ship_kind_name(&ship_kind);
                    debug!("Received JoinRequest: {:?}", ship_kind);

                    {
                        let mut lock = rat_lock.lock().unwrap();
                        if lock.get(&ship_kind).is_some() {
                            // Client is reconnecting - allow it by removing old entry
                            // The old Zenoh subscriber task will eventually exit when it
                            // detects no more messages or errors
                            info!("Client {:?} reconnecting, allowing rejoin", ship_kind);
                            lock.remove(&ship_kind);
                        }
                        lock.insert(ship_kind.clone());
                    }

                    let generated_id = rand::random::<ShipName>().abs();
                    let (disconnect_tx, _disconnect_rx) =
                        tokio::sync::broadcast::channel::<bool>(1);

                    let ship_name_str = match &ship_kind {
                        ShipKind::Rat(name) => name.clone(),
                        ShipKind::Wind(name) => name.clone(),
                    };
                    let ship_name_key = sanitize_key(&ship_name_str);

                    // Create channels for communication with this client
                    let (recv_tx, _) = tokio::sync::broadcast::channel::<(
                        Packet,
                        Option<std::net::SocketAddr>,
                    )>(256);
                    let (send_tx, mut send_rx) = tokio::sync::mpsc::channel::<Packet>(256);

                    let client_addr = crate::NetworkShipAddress {
                        ip: [0, 0, 0, 0],
                        port: 0,
                        ship: generated_id,
                        kind: ship_kind.clone(),
                    };

                    // Key for coordinator -> client messages
                    let coord_to_client_key =
                        format!("minot/{}/coord/clients/{}", domain_id, ship_name_key);
                    // Key for client -> coordinator messages
                    let client_to_coord_key =
                        format!("minot/{}/clients/{}/coord", domain_id, ship_name_key);

                    // Subscriber for receiving from client
                    let client_subscriber = session_clone
                        .declare_subscriber(client_to_coord_key)
                        .wait()
                        .expect("Failed to create subscriber for client");

                    let recv_tx_clone = recv_tx.clone();
                    let ships_lock = std::sync::Arc::clone(&rat_lock);
                    let ship_kind_for_disconnect = ship_kind.clone();

                    // Task to receive from client
                    tokio::spawn(async move {
                        loop {
                            match client_subscriber.recv_async().await {
                                Ok(sample) => {
                                    let payload = sample.payload().to_bytes();
                                    let aligned = align_bytes(&payload);
                                    match from_bytes::<Packet, rancor::Error>(&aligned) {
                                        Ok(packet) => {
                                            if let Err(e) = recv_tx_clone.send((packet, None)) {
                                                debug!("Failed to forward client packet: {}", e);
                                                break;
                                            }
                                        }
                                        Err(e) => {
                                            error!("Failed to deserialize client packet: {}", e);
                                        }
                                    }
                                }
                                Err(e) => {
                                    warn!(
                                        "Client {:?} disconnected: {}",
                                        ship_kind_for_disconnect, e
                                    );
                                    let mut lock = ships_lock.lock().unwrap();
                                    lock.remove(&ship_kind_for_disconnect);
                                    break;
                                }
                            }
                        }
                    });

                    // Create publisher synchronously BEFORE spawning send task
                    // This ensures publisher is ready before we send the welcome
                    debug!("Creating coordinator publisher for {}", coord_to_client_key);
                    let coord_to_client_key_owned = coord_to_client_key.clone();
                    let publisher = session_clone
                        .declare_publisher(coord_to_client_key_owned)
                        .congestion_control(zenoh::qos::CongestionControl::Block)
                        .reliability(zenoh::qos::Reliability::Reliable)
                        .wait()
                        .expect("Failed to create publisher for client");

                    // Send welcome packet directly using the publisher
                    let current_wait_for_ack = { *clwa.read().unwrap() };
                    let welcome_packet = Packet {
                        header: Header {
                            source: CONTROLLER_CLIENT_ID,
                            target: generated_id,
                        },
                        data: PacketKind::Welcome {
                            addr: client_addr.clone(),
                            wait_for_ack: current_wait_for_ack,
                        },
                    };

                    let bytes = rkyv::api::high::to_bytes::<rancor::Error>(&welcome_packet)
                        .expect("Failed to serialize welcome packet");
                    if let Err(e) = publisher.put(&*bytes).wait() {
                        error!("Failed to send welcome packet: {}", e);
                        continue;
                    }

                    debug!("Welcome packet sent to {}", coord_to_client_key);

                    // Task to send subsequent packets to client
                    tokio::spawn(async move {
                        while let Some(packet) = send_rx.recv().await {
                            let bytes = rkyv::api::high::to_bytes::<rancor::Error>(&packet)
                                .expect("Failed to serialize packet");
                            if let Err(e) = publisher.put(&*bytes).wait() {
                                error!("Failed to send to client: {}", e);
                                break;
                            }
                        }
                    });

                    let ship_handle = ShipHandle {
                        ship: generated_id,
                        disconnect: disconnect_tx,
                        recv: recv_tx,
                        send: send_tx,
                        name: ship_kind,
                        addr_from_coord: client_addr,
                        remove_rules_on_disconnect,
                    };

                    if let Err(e) = clients_tx_inner.send(ship_handle) {
                        error!("Failed to broadcast new client: {}", e);
                    }
                    debug!("ShipHandle created and sent");
                }
            }
        });

        Self {
            network_clients_chan: clients_tx,
            session,
            domain_id,
        }
    }

    pub fn pad_string(input: &str) -> String {
        if input.len() >= 64 {
            return input.to_string();
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
        // Zenoh session cleanup is handled automatically when dropped
        info!("Sea coordinator shutting down");
    }
}
