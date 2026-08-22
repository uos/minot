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

/// Delivery mode for a node.
///
/// Two independent axes are encoded here, and they are easy to confuse:
///
/// * **Wire QoS** — how Zenoh carries the bytes ([`Qos::reliability`],
///   [`Qos::congestion_control`]).
/// * **Failure policy** — what the rest of the system does when this node stops
///   answering ([`Qos::is_monitored`], [`Qos::fires_torpedo`],
///   [`Qos::removes_rules_on_exit`]).
///
/// Always ask through the predicates below rather than comparing variants.
/// Matching on `== Qos::Reliable` and `== Qos::BestEffort` spells the same
/// question two opposite ways, so a third variant silently lands on different
/// sides of the two spellings.
#[derive(Serialize, Deserialize, Archive, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Qos {
    /// Reliable wire delivery, and a node whose loss is fatal to the system:
    /// peers heartbeat-monitor it and a Torpedo tears the run down when it dies.
    /// Sends to it block the caller until they land.
    #[default]
    Reliable,
    /// Reliable wire delivery and ordering, without the fatal failure policy.
    ///
    /// Comparable to a ROS 2 DDS `RELIABLE` subscriber: writes are retried and
    /// ordered, a write that cannot land within its deadline fails on its own
    /// rather than taking the participant down, and nothing tears down peers.
    /// Sends are dispatched off the caller's thread, so a slow or roaming link
    /// cannot wedge the publisher's loop. Intended for links where latency is
    /// unpredictable — WiFi viewers, tablets, anything off the LAN.
    TryReliable,
    /// Unreliable single-attempt delivery, dropped under congestion, non-fatal.
    /// For high-rate streams where the next sample is worth more than this one.
    BestEffort,
}

impl Qos {
    /// Wire reliability. Note this is close to free over TCP, which already
    /// retransmits and orders; it bites on lossy transports.
    pub fn reliability(self) -> zenoh::qos::Reliability {
        match self {
            Qos::Reliable | Qos::TryReliable => zenoh::qos::Reliability::Reliable,
            Qos::BestEffort => zenoh::qos::Reliability::BestEffort,
        }
    }

    /// Backpressure behaviour when the transmit queue is full.
    ///
    /// `Reliable` blocks indefinitely, which is what wedges a publisher when the
    /// link stalls. `TryReliable` uses `BlockFirst`: wait for the first message,
    /// drop later ones instead of stalling without bound.
    pub fn congestion_control(self) -> zenoh::qos::CongestionControl {
        match self {
            Qos::Reliable => zenoh::qos::CongestionControl::Block,
            Qos::TryReliable => zenoh::qos::CongestionControl::BlockFirst,
            Qos::BestEffort => zenoh::qos::CongestionControl::Drop,
        }
    }

    /// Whether peers must heartbeat-monitor this node and report it dead.
    pub fn is_monitored(self) -> bool {
        matches!(self, Qos::Reliable)
    }

    /// Whether losing this node fires a Torpedo that stops the whole run.
    pub fn fires_torpedo(self) -> bool {
        matches!(self, Qos::Reliable)
    }

    /// Whether the coordinator drops this node's routing rules when it goes
    /// away. Resilient nodes keep theirs so a reconnect finds its routes intact.
    pub fn removes_rules_on_exit(self) -> bool {
        matches!(self, Qos::Reliable)
    }

    /// Whether delivery to this node is expected not to silently drop samples.
    /// Used to reject pairings with a best-effort publisher.
    pub fn expects_reliable_delivery(self) -> bool {
        matches!(self, Qos::Reliable | Qos::TryReliable)
    }

    /// Whether sends to this node are dispatched off the caller's thread.
    ///
    /// False only for `Reliable`, whose send is awaited inline and retried
    /// without bound. Everything else must not be able to wedge a frame loop.
    pub fn dispatch_is_async(self) -> bool {
        !matches!(self, Qos::Reliable)
    }
}

#[derive(Serialize, Deserialize, Archive, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RatPubRegisterKind {
    Publish,
    Subscribe,
    Scope,
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
        node_mode: Qos,
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
        node_mode: Qos,
    },
    /// Stop delivering a topic to a ship. Without this a subscriber can only
    /// stop reading, and the coordinator keeps pushing the data at it.
    UnregisterShipAtVar {
        ship: String,
        var: String,
        kind: RatPubRegisterKind,
    },
    RegistrationError(String),
    Sonar,
    Torpedo(Vec<String>),
    ClientsHash {
        reliable: HashSet<String>,
        best_effort: HashSet<String>,
    },
    PeerDead {
        ship: String,
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
    pub node_mode: Qos,
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
    ) -> anyhow::Result<Self> {
        let domain_id = get_domain_id();
        if domain_id > 0 {
            info!("Coordinator using domain ID {}", domain_id);
        }

        // Initialize Zenoh session
        let session = crate::network::open_zenoh_session(crate::network::NetworkRole::Coordinator)?;
        let session = std::sync::Arc::new(session);

        let (clients_tx, _clients_rx) = tokio::sync::broadcast::channel::<ShipHandle>(64);
        let clients_tx_inner = clients_tx.clone();

        // Key expression for join requests
        let join_key = format!("minot/{}/coord/join", domain_id);

        // Subscribe to join requests from clients
        let session_clone = std::sync::Arc::clone(&session);
        let clwa = std::sync::Arc::clone(&clients_wait_for_ack);
        let rat_lock = std::sync::Arc::new(std::sync::Mutex::new(HashSet::new()));
        let subscriber = session
            .declare_subscriber(join_key.clone())
            .wait()
            .map_err(|e| anyhow::anyhow!("Failed to create coordinator join subscriber: {e}"))?;

        tokio::spawn(async move {
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
                    node_mode,
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
                        node_mode,
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
                        .congestion_control(node_mode.congestion_control())
                        .reliability(node_mode.reliability())
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
                        node_mode,
                    };

                    if let Err(e) = clients_tx_inner.send(ship_handle) {
                        error!("Failed to broadcast new client: {}", e);
                    }
                    debug!("ShipHandle created and sent");
                }
            }
        });

        Ok(Self {
            network_clients_chan: clients_tx,
            session,
            domain_id,
        })
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

#[cfg(test)]
mod qos_tests {
    use super::Qos;

    /// The two axes are independent, and `TryReliable` is the combination that
    /// only exists because they are: reliable on the wire, non-fatal on failure.
    #[test]
    fn try_reliable_is_reliable_on_the_wire_but_never_fatal() {
        let mode = Qos::TryReliable;

        assert_eq!(mode.reliability(), zenoh::qos::Reliability::Reliable);
        assert!(mode.expects_reliable_delivery());

        assert!(!mode.is_monitored());
        assert!(!mode.fires_torpedo());
        assert!(!mode.removes_rules_on_exit());
    }

    /// Only `Reliable` may take the rest of the system down with it, and only
    /// `Reliable` blocks its caller. Everything else must stay survivable.
    #[test]
    fn reliable_is_the_only_fatal_and_the_only_blocking_mode() {
        for mode in [Qos::TryReliable, Qos::BestEffort] {
            assert!(!mode.fires_torpedo(), "{mode:?} must not fire a Torpedo");
            assert!(!mode.is_monitored(), "{mode:?} must not be monitored");
            assert!(
                mode.dispatch_is_async(),
                "{mode:?} must not be able to wedge its caller"
            );
        }

        assert!(Qos::Reliable.fires_torpedo());
        assert!(Qos::Reliable.is_monitored());
        assert!(!Qos::Reliable.dispatch_is_async());
    }

    /// `Block` is what stalls a publisher when a link goes away. Only the mode
    /// that accepts being wedged may use it.
    #[test]
    fn only_reliable_blocks_without_bound_under_congestion() {
        use zenoh::qos::CongestionControl;

        assert_eq!(Qos::Reliable.congestion_control(), CongestionControl::Block);
        assert_eq!(
            Qos::TryReliable.congestion_control(),
            CongestionControl::BlockFirst
        );
        assert_eq!(
            Qos::BestEffort.congestion_control(),
            CongestionControl::Drop
        );
    }

    /// A best-effort publisher drops samples, so any subscriber that expects
    /// delivery has to be rejected rather than silently starved.
    #[test]
    fn best_effort_is_the_only_mode_that_tolerates_dropped_samples() {
        assert!(Qos::Reliable.expects_reliable_delivery());
        assert!(Qos::TryReliable.expects_reliable_delivery());
        assert!(!Qos::BestEffort.expects_reliable_delivery());
    }

    /// `Reliable` is the default, so an unknown or departed client is treated
    /// as fatal rather than silently downgraded.
    #[test]
    fn default_mode_is_reliable() {
        assert_eq!(Qos::default(), Qos::Reliable);
    }
}
