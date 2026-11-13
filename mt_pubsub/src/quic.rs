//! Experimental peer transport on QUIC (quinn), without Zenoh.
//!
//! Every node owns one UDP socket and keeps one QUIC connection to every other
//! node (full mesh). Per connection:
//!
//! * the first unidirectional stream of each side is its control stream: hello,
//!   known peers (gossip), and the writers and readers it declares. A closed
//!   connection withdraws everything its node declared.
//! * BestEffort samples travel as QUIC datagrams when they fit, otherwise on
//!   one unidirectional stream per sample.
//! * TryReliable and Reliable samples travel in order on one bidirectional
//!   stream per publisher; Reliable samples are acknowledged after enqueue.
//!
//! * Samples of 16 KiB and more reach subscribers on the same host through
//!   shared memory instead (`quic_shm.rs`): best-effort ones entirely, ordered
//!   ones with a token on the publisher's stream in place of the payload, so
//!   order and acknowledgements stay with QUIC.
//!
//! Connections are plain by default. With encryption on they are encrypted
//! but not authenticated: certificates are self-signed and never verified.
use super::NodeConfig;
use crate::peer::{ArchivedWireSample, Outgoing, SharedBuffer, aligned, qos_code, topic_key};
use crate::quic_shm as shm;
use anyhow::{Context, Result, anyhow, bail, ensure};
use bytes::{BufMut, Bytes, BytesMut};
use mt_sea::{ArchivedMessage, Qos, Reliability, Sendable};
use rkyv::util::AlignedVec;
use std::{
    collections::{HashMap, HashSet},
    hash::{BuildHasher, Hasher},
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr, SocketAddrV4},
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
    time::Duration,
};
use tokio::sync::{mpsc, oneshot};
use tokio_util::sync::CancellationToken;

const ALPN: &[u8] = b"minot-pubsub/1";
const BEACON_MAGIC: &[u8; 4] = b"MNQ1";
/// Not the Zenoh scouting group, so both transports can share a network.
const BEACON_GROUP: Ipv4Addr = Ipv4Addr::new(224, 0, 0, 225);
const BEACON_INTERVAL: Duration = Duration::from_secs(1);
/// Buffer between the connection tasks and a subscriber's own queue.
const SINK_DEPTH: usize = 64;
const MAX_SAMPLE: usize = 1 << 30;

fn env_ms(name: &str, default: u64) -> Duration {
    Duration::from_millis(
        std::env::var(name)
            .ok()
            .and_then(|value| value.parse().ok())
            .filter(|ms| *ms > 0)
            .unwrap_or(default),
    )
}
/// A connection that stays silent this long is dropped and dialed again.
/// QUIC backs its probes off exponentially during an outage, so an old
/// connection can stay quiet for seconds after the peer is back in range,
/// while a new handshake takes one round trip.
fn idle_timeout() -> Duration {
    env_ms("MINOT_QUIC_IDLE_MS", 3000)
}
fn key_hash(key: &str) -> u64 {
    key.bytes().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
    })
}
fn parse_endpoint(endpoint: &str) -> Result<SocketAddr> {
    let address = endpoint.rsplit('/').next().unwrap_or(endpoint);
    address
        .parse()
        .with_context(|| format!("invalid QUIC endpoint '{endpoint}'"))
}

#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Debug, Clone)]
enum Control {
    Hello {
        id: u64,
        domain: u16,
        mode: u8,
        peers: Vec<(u64, String)>,
        /// Shared memory socket of the node, empty without one.
        shm: String,
    },
    Peers(Vec<(u64, String)>),
    Declare {
        endpoint: String,
        reader: bool,
        key: String,
        qos: u8,
    },
    Undeclare(String),
    Torpedo(String),
}
async fn write_control(stream: &mut quinn::SendStream, control: &Control) -> Result<()> {
    let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(control)?;
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .await?;
    stream.write_all(&bytes).await?;
    Ok(())
}
async fn read_control(stream: &mut quinn::RecvStream) -> Result<Control> {
    let mut len = [0u8; 4];
    stream.read_exact(&mut len).await?;
    let len = u32::from_le_bytes(len) as usize;
    ensure!(len <= 1 << 20, "oversized control frame");
    let mut bytes = vec![0u8; len];
    stream.read_exact(&mut bytes).await?;
    Ok(rkyv::from_bytes::<Control, rkyv::rancor::Error>(&aligned(
        len,
        [bytes.as_slice()],
    ))?)
}

#[derive(Debug)]
struct NoVerify(Arc<rustls::crypto::CryptoProvider>);
impl rustls::client::danger::ServerCertVerifier for NoVerify {
    fn verify_server_cert(
        &self,
        _: &rustls::pki_types::CertificateDer<'_>,
        _: &[rustls::pki_types::CertificateDer<'_>],
        _: &rustls::pki_types::ServerName<'_>,
        _: &[u8],
        _: rustls::pki_types::UnixTime,
    ) -> Result<rustls::client::danger::ServerCertVerified, rustls::Error> {
        Ok(rustls::client::danger::ServerCertVerified::assertion())
    }
    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &rustls::pki_types::CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls12_signature(
            message,
            cert,
            dss,
            &self.0.signature_verification_algorithms,
        )
    }
    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &rustls::pki_types::CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls13_signature(
            message,
            cert,
            dss,
            &self.0.signature_verification_algorithms,
        )
    }
    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        self.0.signature_verification_algorithms.supported_schemes()
    }
}

fn quic_configs(encrypt: bool) -> Result<(quinn::ServerConfig, quinn::ClientConfig)> {
    let idle = idle_timeout();
    let mut transport = quinn::TransportConfig::default();
    transport
        .max_idle_timeout(Some(idle.try_into()?))
        .keep_alive_interval(Some(idle / 3))
        .max_concurrent_uni_streams(1024u32.into())
        .max_concurrent_bidi_streams(1024u32.into())
        .stream_receive_window((8u32 << 20).into())
        .send_window(32 << 20)
        .datagram_receive_buffer_size(Some(4 << 20))
        .datagram_send_buffer_size(1 << 20);
    let transport = Arc::new(transport);

    let (mut server, mut client) = if encrypt {
        tls_configs()?
    } else {
        (
            crate::quic_plain::server_config(),
            crate::quic_plain::client_config(),
        )
    };
    server.transport_config(transport.clone());
    client.transport_config(transport);
    Ok((server, client))
}

/// TLS 1.3 with a certificate made up on the spot and accepted unchecked:
/// encrypted against listeners, but any node may join.
fn tls_configs() -> Result<(quinn::ServerConfig, quinn::ClientConfig)> {
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    let rcgen::CertifiedKey { cert, signing_key } =
        rcgen::generate_simple_self_signed(vec!["minot".to_owned()])?;
    let key = rustls::pki_types::PrivatePkcs8KeyDer::from(signing_key.serialize_der());
    let mut server = rustls::ServerConfig::builder_with_provider(provider.clone())
        .with_protocol_versions(&[&rustls::version::TLS13])?
        .with_no_client_auth()
        .with_single_cert(vec![cert.der().clone()], key.into())?;
    server.alpn_protocols = vec![ALPN.to_vec()];
    let mut client = rustls::ClientConfig::builder_with_provider(provider.clone())
        .with_protocol_versions(&[&rustls::version::TLS13])?
        .dangerous()
        .with_custom_certificate_verifier(Arc::new(NoVerify(provider)))
        .with_no_client_auth();
    client.alpn_protocols = vec![ALPN.to_vec()];

    let server = quinn::ServerConfig::with_crypto(Arc::new(
        quinn::crypto::rustls::QuicServerConfig::try_from(server)?,
    ));
    let client = quinn::ClientConfig::new(Arc::new(
        quinn::crypto::rustls::QuicClientConfig::try_from(client)?,
    ));
    Ok((server, client))
}

#[derive(Debug, Clone)]
struct Decl {
    reader: bool,
    key: String,
    qos: u8,
}
#[derive(Debug)]
struct Peer {
    conn: quinn::Connection,
    /// Node that opened the connection; decides which of two duplicates stays.
    dialer: u64,
    mode: u8,
    control: mpsc::UnboundedSender<Control>,
    decls: HashMap<String, Decl>,
    /// Set for a peer on this host.
    shm: Option<Arc<shm::Link>>,
}
#[derive(Debug)]
struct Incoming {
    bytes: Bytes,
    ack: Option<oneshot::Sender<bool>>,
}
#[derive(Debug)]
struct Sink {
    id: String,
    reliable: bool,
    tx: mpsc::Sender<Incoming>,
}
type Retained = Arc<Mutex<Option<Bytes>>>;
/// A peer with a subscriber: identifier, connection, shared memory link.
type Reader = (u64, quinn::Connection, Option<Arc<shm::Link>>);
/// In the length field of an ordered sample: a shared memory token follows
/// instead of the payload.
const SHARED: u32 = u32::MAX;
#[derive(Debug, Default)]
struct State {
    peers: HashMap<u64, Peer>,
    dialing: HashSet<SocketAddr>,
    /// Writers and readers of this node, by endpoint identifier.
    decls: HashMap<String, Decl>,
    /// Subscribers of this node, by topic key hash.
    sinks: HashMap<u64, Vec<Sink>>,
    /// Last sample of every transient-local writer of this node.
    retained: HashMap<String, (String, Retained)>,
}

#[derive(Debug)]
struct Inner {
    id: u64,
    domain: u16,
    mode: u8,
    timeout: Duration,
    endpoint: quinn::Endpoint,
    shutdown: CancellationToken,
    state: Mutex<State>,
    next: AtomicU64,
    shm: Option<Arc<shm::Hub>>,
}

async fn send_sample_stream(
    conn: quinn::Connection,
    hash: u64,
    bytes: Bytes,
    budget: Duration,
) -> Result<()> {
    let mut stream = conn.open_uni().await?;
    let sent = tokio::time::timeout(budget, async {
        stream.write_all(&hash.to_le_bytes()).await?;
        stream.write_chunk(bytes).await?;
        stream.finish()?;
        // Resolves once the peer has acknowledged the whole sample.
        stream.stopped().await?;
        anyhow::Ok(())
    })
    .await;
    match sent {
        Ok(result) => result,
        Err(_) => {
            let _ = stream.reset(0u32.into());
            bail!("sample stream timed out")
        }
    }
}

impl Inner {
    fn identifier(&self) -> String {
        format!(
            "{:016x}_{}",
            self.id,
            self.next.fetch_add(1, Ordering::Relaxed)
        )
    }
    fn declare(&self, endpoint: &str, decl: Decl) {
        let mut state = self.state.lock().unwrap();
        for peer in state.peers.values() {
            let _ = peer.control.send(Control::Declare {
                endpoint: endpoint.to_owned(),
                reader: decl.reader,
                key: decl.key.clone(),
                qos: decl.qos,
            });
        }
        state.decls.insert(endpoint.to_owned(), decl);
    }
    fn undeclare(&self, endpoint: &str) {
        let mut state = self.state.lock().unwrap();
        state.retained.remove(endpoint);
        if state.decls.remove(endpoint).is_some() {
            for peer in state.peers.values() {
                let _ = peer.control.send(Control::Undeclare(endpoint.to_owned()));
            }
        }
    }
    /// Endpoints declared by this node and by connected peers.
    fn count(&self, key: &str, reader: bool, qos: Option<u8>) -> usize {
        let state = self.state.lock().unwrap();
        let matches = |decl: &&Decl| {
            decl.reader == reader && decl.key == key && qos.is_none_or(|qos| decl.qos == qos)
        };
        state.decls.values().filter(matches).count()
            + state
                .peers
                .values()
                .map(|peer| peer.decls.values().filter(matches).count())
                .sum::<usize>()
    }
    /// Connected peers with a subscriber for `key`.
    fn readers(&self, key: &str) -> Vec<Reader> {
        let state = self.state.lock().unwrap();
        state
            .peers
            .iter()
            .filter(|(_, peer)| peer.decls.values().any(|d| d.reader && d.key == key))
            .map(|(id, peer)| (*id, peer.conn.clone(), peer.shm.clone()))
            .collect()
    }
    fn sinks(&self, hash: u64) -> Vec<mpsc::Sender<Incoming>> {
        let state = self.state.lock().unwrap();
        state
            .sinks
            .get(&hash)
            .map(|sinks| sinks.iter().map(|sink| sink.tx.clone()).collect())
            .unwrap_or_default()
    }
    fn deliver_lossy(&self, hash: u64, bytes: &Bytes) {
        for sink in self.sinks(hash) {
            let _ = sink.try_send(Incoming {
                bytes: bytes.clone(),
                ack: None,
            });
        }
    }
    /// Hand a sample to every local subscriber in order. With `ack`, wait until
    /// each has enqueued it; a subscriber that went away counts as served.
    async fn deliver_ordered(&self, hash: u64, bytes: &Bytes, ack: bool) -> bool {
        let mut pending = Vec::new();
        for sink in self.sinks(hash) {
            let (tx, rx) = oneshot::channel();
            let incoming = Incoming {
                bytes: bytes.clone(),
                ack: ack.then_some(tx),
            };
            if sink.send(incoming).await.is_ok() && ack {
                pending.push(rx);
            }
        }
        let mut accepted = true;
        for reply in pending {
            accepted &= reply.await.unwrap_or(true);
        }
        accepted
    }
    async fn fail(&self, message: &str) {
        {
            let state = self.state.lock().unwrap();
            for peer in state.peers.values() {
                let _ = peer.control.send(Control::Torpedo(message.to_owned()));
            }
        }
        // Closing the endpoint discards unsent stream data.
        tokio::time::sleep(Duration::from_millis(50)).await;
        self.shutdown.cancel();
    }

    fn connected(&self, address: SocketAddr) -> bool {
        let state = self.state.lock().unwrap();
        state.dialing.contains(&address)
            || state
                .peers
                .values()
                .any(|peer| peer.conn.remote_address() == address)
    }
    /// One socket serves both directions, so the address a peer is seen at is
    /// also the address it can be dialed at.
    fn learn(self: &Arc<Self>, id: u64, address: SocketAddr) {
        // The lower identifier dials, so two nodes that learn of each other at
        // the same time open one connection.
        if self.id >= id || self.state.lock().unwrap().peers.contains_key(&id) {
            return;
        }
        self.dial(address);
    }
    fn dial(self: &Arc<Self>, address: SocketAddr) {
        if self.shutdown.is_cancelled() || !self.state.lock().unwrap().dialing.insert(address) {
            return;
        }
        let node = Arc::clone(self);
        tokio::spawn(async move {
            let attempt = async {
                let connecting = node.endpoint.connect(address, "minot")?;
                anyhow::Ok(
                    tokio::time::timeout(idle_timeout(), connecting)
                        .await
                        .context("handshake timed out")??,
                )
            };
            let result = attempt.await;
            match result {
                Ok(conn) => {
                    // Still marked as dialing until the peer is registered.
                    let served = node.serve(conn, true, Some(address)).await;
                    if let Err(error) = served {
                        log::debug!("QUIC connection to {address} ended: {error:#}");
                    }
                }
                Err(error) => {
                    node.state.lock().unwrap().dialing.remove(&address);
                    log::debug!("QUIC dial {address} failed: {error:#}");
                }
            }
        });
    }

    async fn serve(
        self: &Arc<Self>,
        conn: quinn::Connection,
        dialed: bool,
        dialing: Option<SocketAddr>,
    ) -> Result<()> {
        let greeting = self.greet(&conn).await;
        if let Some(address) = dialing {
            self.state.lock().unwrap().dialing.remove(&address);
        }
        let (mut control_tx, mut control_rx, remote, mode, peers, shm) = match greeting {
            Ok(greeting) => greeting,
            Err(error) => {
                conn.close(1u32.into(), b"handshake");
                return Err(error);
            }
        };
        let dialer = if dialed { self.id } else { remote };
        let (queue, mut queued) = mpsc::unbounded_channel();
        {
            let mut state = self.state.lock().unwrap();
            if let Some(old) = state.peers.get(&remote) {
                // Both nodes apply the same rule, so they keep the same one.
                // Equal dialers mean a redial: the dialer has given up on the
                // old connection.
                if old.dialer < dialer && old.conn.close_reason().is_none() {
                    conn.close(1u32.into(), b"duplicate");
                    return Ok(());
                }
                old.conn.close(1u32.into(), b"duplicate");
            }
            for (endpoint, decl) in &state.decls {
                let _ = queue.send(Control::Declare {
                    endpoint: endpoint.clone(),
                    reader: decl.reader,
                    key: decl.key.clone(),
                    qos: decl.qos,
                });
            }
            let news = vec![(remote, conn.remote_address().to_string())];
            let mut known = Vec::new();
            for (id, peer) in &state.peers {
                if *id != remote {
                    let _ = peer.control.send(Control::Peers(news.clone()));
                    known.push((*id, peer.conn.remote_address().to_string()));
                }
            }
            // The hello listed the peers known before it was sent. A node that
            // registered since would otherwise stay unknown to this one, and
            // only the lower identifier of a pair dials.
            let _ = queue.send(Control::Peers(known));
            state.peers.insert(
                remote,
                Peer {
                    conn: conn.clone(),
                    dialer,
                    mode,
                    control: queue,
                    decls: HashMap::new(),
                    shm: None,
                },
            );
        }
        log::debug!(
            "QUIC peer {remote:016x} connected at {}",
            conn.remote_address()
        );
        self.learn_all(peers);
        if self.shm.is_some() {
            // Not awaited before the peer is registered: until then the
            // connection counts as missing and would be dialed again.
            let (node, conn) = (Arc::clone(self), conn.clone());
            tokio::spawn(async move {
                let link = shm::Link::connect(&shm, node.id).await;
                let mut state = node.state.lock().unwrap();
                if let Some(peer) = state.peers.get_mut(&remote)
                    && peer.conn.stable_id() == conn.stable_id()
                {
                    peer.shm = link;
                }
            });
        }

        let writer = conn.clone();
        tokio::spawn(async move {
            while let Some(control) = queued.recv().await {
                if write_control(&mut control_tx, &control).await.is_err() {
                    writer.close(1u32.into(), b"control");
                    break;
                }
            }
        });
        let node = Arc::clone(self);
        let reader = conn.clone();
        tokio::spawn(async move {
            while let Ok(control) = read_control(&mut control_rx).await {
                node.on_control(remote, &reader, control);
            }
            reader.close(1u32.into(), b"control");
        });

        loop {
            tokio::select! {
                _ = self.shutdown.cancelled() => break,
                _ = conn.closed() => break,
                datagram = conn.read_datagram() => {
                    let Ok(bytes) = datagram else { break; };
                    if bytes.len() > 8 {
                        let hash = u64::from_le_bytes(bytes[..8].try_into().unwrap());
                        self.deliver_lossy(hash, &bytes.slice(8..));
                    }
                }
                stream = conn.accept_uni() => {
                    let Ok(mut stream) = stream else { break; };
                    let node = Arc::clone(self);
                    let nudge = conn.clone();
                    tokio::spawn(async move {
                        let Ok(bytes) = stream.read_to_end(MAX_SAMPLE).await else { return; };
                        // The publisher sends its next sample once this one is
                        // acknowledged, but QUIC may hold the ACK of the last
                        // packet for its delayed-ACK timer (25 ms). Any packet
                        // carries pending ACKs along, so send an empty one.
                        let _ = nudge.send_datagram(Bytes::from_static(&[0]));
                        if bytes.len() > 8 {
                            let hash = u64::from_le_bytes(bytes[..8].try_into().unwrap());
                            node.deliver_lossy(hash, &Bytes::from(bytes).slice(8..));
                        }
                    });
                }
                stream = conn.accept_bi() => {
                    let Ok((send, recv)) = stream else { break; };
                    let node = Arc::clone(self);
                    tokio::spawn(async move { let _ = node.ordered(remote, send, recv).await; });
                }
            }
        }
        conn.close(0u32.into(), b"");
        let lost = {
            let mut state = self.state.lock().unwrap();
            match state.peers.get(&remote) {
                Some(peer) if peer.conn.stable_id() == conn.stable_id() => {
                    state.peers.remove(&remote)
                }
                _ => None,
            }
        };
        if let Some(peer) = lost {
            log::debug!("QUIC peer {remote:016x} lost: {:?}", conn.close_reason());
            if peer.mode == 1 && !self.shutdown.is_cancelled() {
                log::warn!("Reliable pub/sub peer disappeared: {remote:016x}");
                self.shutdown.cancel();
            }
        }
        Ok(())
    }
    /// Exchange hellos on the control streams.
    async fn greet(
        &self,
        conn: &quinn::Connection,
    ) -> Result<(
        quinn::SendStream,
        quinn::RecvStream,
        u64,
        u8,
        Vec<(u64, String)>,
        String,
    )> {
        let exchange = async {
            let mut send = conn.open_uni().await?;
            let peers = {
                let state = self.state.lock().unwrap();
                state
                    .peers
                    .iter()
                    .map(|(id, peer)| (*id, peer.conn.remote_address().to_string()))
                    .collect()
            };
            let hello = Control::Hello {
                id: self.id,
                domain: self.domain,
                mode: self.mode,
                peers,
                shm: self.shm.as_ref().map(|hub| hub.path()).unwrap_or_default(),
            };
            write_control(&mut send, &hello).await?;
            // Streams are accepted in the order the peer opened them.
            let mut recv = conn.accept_uni().await?;
            let Control::Hello {
                id,
                domain,
                mode,
                peers,
                shm,
            } = read_control(&mut recv).await?
            else {
                bail!("peer did not start with a hello");
            };
            ensure!(domain == self.domain, "peer is in domain {domain}");
            ensure!(id != self.id, "connected to self");
            Ok((send, recv, id, mode, peers, shm))
        };
        tokio::time::timeout(idle_timeout(), exchange)
            .await
            .context("hello timed out")?
    }
    fn learn_all(self: &Arc<Self>, peers: Vec<(u64, String)>) {
        for (id, address) in peers {
            if let Ok(address) = address.parse() {
                self.learn(id, address);
            }
        }
    }
    fn on_control(self: &Arc<Self>, remote: u64, conn: &quinn::Connection, control: Control) {
        match control {
            Control::Hello { .. } => {}
            Control::Peers(peers) => self.learn_all(peers),
            Control::Torpedo(message) => {
                log::warn!("pub/sub peer failed: {message}");
                self.shutdown.cancel();
            }
            Control::Undeclare(endpoint) => {
                let mut state = self.state.lock().unwrap();
                if let Some(peer) = state.peers.get_mut(&remote)
                    && peer.conn.stable_id() == conn.stable_id()
                {
                    peer.decls.remove(&endpoint);
                }
            }
            Control::Declare {
                endpoint,
                reader,
                key,
                qos,
            } => {
                let mut state = self.state.lock().unwrap();
                let Some(peer) = state.peers.get_mut(&remote) else {
                    return;
                };
                if peer.conn.stable_id() != conn.stable_id() {
                    return;
                }
                peer.decls.insert(
                    endpoint,
                    Decl {
                        reader,
                        key: key.clone(),
                        qos,
                    },
                );
                let hash = key_hash(&key);
                if reader {
                    // A late subscriber gets the retained samples pushed.
                    // Sequence numbers deduplicate overlap with live data.
                    for (retained_key, sample) in state.retained.values() {
                        if *retained_key != key {
                            continue;
                        }
                        if let Some(bytes) = sample.lock().unwrap().clone() {
                            tokio::spawn(send_sample_stream(
                                conn.clone(),
                                hash,
                                bytes,
                                self.timeout,
                            ));
                        }
                    }
                } else if qos == 0
                    && let Some(sinks) = state.sinks.get_mut(&hash)
                    && sinks.iter().any(|sink| sink.reliable)
                {
                    log::warn!("BestEffort publisher cannot satisfy a reliable subscriber");
                    sinks.retain(|sink| !sink.reliable);
                }
            }
        }
    }
    /// Receive one publisher's ordered samples and acknowledge the reliable ones.
    async fn ordered(
        &self,
        remote: u64,
        mut send: quinn::SendStream,
        mut recv: quinn::RecvStream,
    ) -> Result<()> {
        loop {
            let mut header = [0u8; 13];
            recv.read_exact(&mut header).await?;
            let len = u32::from_le_bytes(header[..4].try_into().unwrap());
            let ack = header[4] == 1;
            let hash = u64::from_le_bytes(header[5..].try_into().unwrap());
            let bytes = if len == SHARED {
                let mut token = [0u8; 8];
                recv.read_exact(&mut token).await?;
                let token = u64::from_le_bytes(token);
                // Sent before its token, so normally here already.
                let hub = self.shm.as_ref().context("no shared memory")?;
                hub.take(remote, token, self.timeout)
                    .await
                    .context("shared sample did not arrive")?
            } else {
                ensure!(len as usize <= MAX_SAMPLE, "oversized sample");
                let mut bytes = BytesMut::zeroed(len as usize);
                recv.read_exact(&mut bytes).await?;
                bytes.freeze()
            };
            let accepted = self.deliver_ordered(hash, &bytes, ack).await;
            if ack {
                send.write_all(&[u8::from(accepted)]).await?;
            }
        }
    }

    async fn accept(self: Arc<Self>) {
        loop {
            tokio::select! {
                _ = self.shutdown.cancelled() => break,
                incoming = self.endpoint.accept() => {
                    let Some(incoming) = incoming else { break; };
                    let node = Arc::clone(&self);
                    tokio::spawn(async move {
                        if let Ok(conn) = incoming.await {
                            let _ = node.serve(conn, false, None).await;
                        }
                    });
                }
            }
        }
        self.endpoint.close(0u32.into(), b"shutdown");
    }
    /// Keep a connection to every configured bootstrap peer.
    async fn bootstrap(self: Arc<Self>, endpoints: Vec<SocketAddr>) {
        let mut tick = tokio::time::interval(BEACON_INTERVAL);
        loop {
            tokio::select! {
                _ = self.shutdown.cancelled() => break,
                _ = tick.tick() => {
                    for address in &endpoints {
                        if !self.connected(*address) {
                            self.dial(*address);
                        }
                    }
                }
            }
        }
    }
    /// Announce this node by multicast and dial the nodes that announce
    /// themselves. A node that returns into range is found by its next beacon.
    async fn beacon(self: Arc<Self>, socket: tokio::net::UdpSocket, port: u16) {
        let group = SocketAddr::from((BEACON_GROUP, port));
        let mut packet = Vec::with_capacity(16);
        packet.extend_from_slice(BEACON_MAGIC);
        packet.extend_from_slice(&self.domain.to_le_bytes());
        packet.extend_from_slice(&self.id.to_le_bytes());
        let quic_port = self.endpoint.local_addr().map(|a| a.port()).unwrap_or(0);
        packet.extend_from_slice(&quic_port.to_le_bytes());
        let mut tick = tokio::time::interval(BEACON_INTERVAL);
        let mut buffer = [0u8; 64];
        loop {
            tokio::select! {
                _ = self.shutdown.cancelled() => break,
                _ = tick.tick() => { let _ = socket.send_to(&packet, group).await; }
                received = socket.recv_from(&mut buffer) => {
                    let Ok((len, source)) = received else { continue; };
                    let beacon = &buffer[..len];
                    if len != 16 || &beacon[..4] != BEACON_MAGIC {
                        continue;
                    }
                    let domain = u16::from_le_bytes(beacon[4..6].try_into().unwrap());
                    let id = u64::from_le_bytes(beacon[6..14].try_into().unwrap());
                    let port = u16::from_le_bytes(beacon[14..16].try_into().unwrap());
                    if domain == self.domain && id != self.id {
                        self.learn(id, SocketAddr::new(source.ip(), port));
                    }
                }
            }
        }
    }
}

fn beacon_socket(port: u16, local: bool) -> Result<tokio::net::UdpSocket> {
    use socket2::{Domain, Protocol, Socket, Type};
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    // Several nodes on one host all listen for beacons.
    socket.set_reuse_address(true)?;
    #[cfg(unix)]
    socket.set_reuse_port(true)?;
    socket.set_nonblocking(true)?;
    socket.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port).into())?;
    let interface = if local {
        Ipv4Addr::LOCALHOST
    } else {
        Ipv4Addr::UNSPECIFIED
    };
    socket.join_multicast_v4(&BEACON_GROUP, &interface)?;
    socket.set_multicast_loop_v4(true)?;
    if local {
        socket.set_multicast_if_v4(&interface)?;
    }
    Ok(tokio::net::UdpSocket::from_std(socket.into())?)
}

#[derive(Debug)]
pub struct QuicNode {
    pub(crate) mode: Qos,
    pub(crate) shutdown: CancellationToken,
    pub(crate) connection: Arc<mt_sea::ConnectionState>,
    inner: Arc<Inner>,
}
impl Drop for QuicNode {
    fn drop(&mut self) {
        self.shutdown.cancel();
    }
}
impl QuicNode {
    pub(crate) async fn create(config: &NodeConfig) -> Result<Arc<Self>> {
        config.node_options().timing.validate()?;
        let local = mt_sea::network::is_local_only();
        let listen = match &config.listen_endpoint {
            Some(endpoint) => parse_endpoint(endpoint)?,
            None if local => SocketAddr::from((Ipv4Addr::LOCALHOST, 0)),
            None => SocketAddr::from((Ipv4Addr::UNSPECIFIED, 0)),
        };
        let (server, client) = quic_configs(config.quic_encrypted())?;
        let mut endpoint = quinn::Endpoint::server(server, listen)
            .with_context(|| format!("cannot bind QUIC endpoint {listen}"))?;
        endpoint.set_default_client_config(client);

        let shutdown = CancellationToken::new();
        // Unique per node: the standard library seeds every hasher randomly.
        // `MINOT_QUIC_ID` fixes it, and so which of two nodes dials, for tests.
        let id = std::env::var("MINOT_QUIC_ID")
            .ok()
            .and_then(|id| id.parse().ok())
            .unwrap_or_else(|| {
                std::collections::hash_map::RandomState::new()
                    .build_hasher()
                    .finish()
            });
        let inner = Arc::new(Inner {
            id,
            domain: mt_sea::net::get_domain_id(),
            mode: qos_code(config.node_mode()),
            timeout: config.node_options().timing.registration_timeout(),
            endpoint,
            shutdown: shutdown.clone(),
            state: Mutex::default(),
            next: AtomicU64::new(1),
            shm: shm::Hub::open(id),
        });
        tokio::spawn(Arc::clone(&inner).accept());
        if let Some(hub) = &inner.shm {
            let node = Arc::clone(&inner);
            let deliver = move |hash, bytes| node.deliver_lossy(hash, &bytes);
            tokio::spawn(Arc::clone(hub).run(shutdown.clone(), deliver));
        }

        let endpoints = config.peer_endpoints.clone().or_else(|| {
            std::env::var("MINOT_PEERS").ok().map(|value| {
                value
                    .split(',')
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .map(str::to_owned)
                    .collect()
            })
        });
        if let Some(endpoints) = endpoints {
            let endpoints = endpoints
                .iter()
                .map(|endpoint| parse_endpoint(endpoint))
                .collect::<Result<Vec<_>>>()?;
            tokio::spawn(Arc::clone(&inner).bootstrap(endpoints));
        } else if !mt_sea::network::is_unicast_only() {
            let port = mt_sea::network::coordinator_port();
            match beacon_socket(port, local) {
                Ok(socket) => {
                    tokio::spawn(Arc::clone(&inner).beacon(socket, port));
                }
                Err(error) => log::warn!("QUIC peer discovery is unavailable: {error:#}"),
            }
        }
        Ok(Arc::new(Self {
            mode: config.node_mode(),
            connection: mt_sea::ConnectionState::for_session(shutdown.clone()),
            shutdown,
            inner,
        }))
    }
    fn check_alive(&self) -> Result<()> {
        ensure!(!self.shutdown.is_cancelled(), "pub/sub node is shut down");
        Ok(())
    }
}

#[derive(Debug)]
struct Channel {
    connection: usize,
    send: quinn::SendStream,
    recv: quinn::RecvStream,
}
#[derive(Debug, Default)]
struct Sending {
    /// Send buffer, kept for reuse.
    spare: Option<AlignedVec>,
    /// Ordered stream to every peer with a subscriber.
    channels: HashMap<u64, Channel>,
    /// Large best-effort samples going out to each peer.
    streams: HashMap<u64, Arc<Mutex<StreamSlot>>>,
}
/// One large best-effort sample in flight to a peer, and the newest one
/// waiting behind it. A waiting sample is replaced by a newer one, so a slow
/// peer gets the latest sample next instead of a backlog.
#[derive(Debug, Default)]
struct StreamSlot {
    busy: bool,
    next: Option<(quinn::Connection, Bytes)>,
}
#[derive(Debug)]
struct PublisherState {
    node: Arc<QuicNode>,
    key: String,
    hash: u64,
    writer: String,
    qos: Qos,
    sequence: AtomicU64,
    /// Serializes concurrent calls through clones to keep per-writer ordering.
    serial: tokio::sync::Mutex<Sending>,
    retained: Retained,
}
impl Drop for PublisherState {
    fn drop(&mut self) {
        self.node.inner.undeclare(&self.writer);
    }
}
#[derive(Debug)]
pub struct QuicPublisher<T> {
    state: Arc<PublisherState>,
    _type: PhantomData<fn() -> T>,
}
impl<T> Clone for QuicPublisher<T> {
    fn clone(&self) -> Self {
        Self {
            state: Arc::clone(&self.state),
            _type: PhantomData,
        }
    }
}
impl<T: Sendable> QuicPublisher<T> {
    pub(crate) async fn create(node: Arc<QuicNode>, topic: String, qos: Qos) -> Result<Self> {
        node.check_alive()?;
        ensure!(!topic.is_empty(), "topic must not be empty");
        let inner = &node.inner;
        let key = topic_key::<T>(inner.domain, &topic);
        if qos_code(qos) == 0 && inner.count(&key, true, Some(1)) > 0 {
            bail!("BestEffort publisher cannot satisfy a Reliable subscriber on {topic}");
        }
        let writer = inner.identifier();
        let retained = Arc::new(Mutex::new(None));
        if qos.is_transient_local() {
            inner
                .state
                .lock()
                .unwrap()
                .retained
                .insert(writer.clone(), (key.clone(), Arc::clone(&retained)));
        }
        inner.declare(
            &writer,
            Decl {
                reader: false,
                key: key.clone(),
                qos: qos_code(qos),
            },
        );
        Ok(Self {
            state: Arc::new(PublisherState {
                hash: key_hash(&key),
                node,
                key,
                writer,
                qos,
                sequence: AtomicU64::new(0),
                serial: tokio::sync::Mutex::default(),
                retained,
            }),
            _type: PhantomData,
        })
    }
    pub(crate) fn subscriber_count(&self) -> usize {
        self.state.node.inner.count(&self.state.key, true, None)
    }
    pub(crate) async fn publish(&self, value: &T) -> Result<()> {
        let state = &self.state;
        state.node.check_alive()?;
        let mut sending = state.serial.lock().await;
        let mut buffer = sending.spare.take().unwrap_or_default();
        buffer.clear();
        let sample = Outgoing {
            writer: &state.writer,
            sequence: state.sequence.fetch_add(1, Ordering::Relaxed),
            qos: qos_code(state.qos),
            value,
        };
        let buffer = Arc::new(
            rkyv::api::high::to_bytes_in::<_, rkyv::rancor::Error>(&sample, buffer)
                .context("Cannot serialize pub/sub message")?,
        );
        let bytes = Bytes::from_owner(SharedBuffer(buffer.clone()));
        let result = self.send(&mut sending, bytes).await;
        // Ours again unless a retained copy, a local subscriber or a sample
        // stream still holds it; the next publish then allocates.
        if let Ok(buffer) = Arc::try_unwrap(buffer) {
            sending.spare = Some(buffer);
        }
        result
    }
    async fn send(&self, sending: &mut Sending, bytes: Bytes) -> Result<()> {
        let state = &self.state;
        let inner = &state.node.inner;
        if state.qos.is_transient_local() {
            *state.retained.lock().unwrap() = Some(bytes.clone());
        }
        let readers = inner.readers(&state.key);
        let mut shared = shm::Lazy::new(&bytes);
        if state.qos.reliability_mode() == Reliability::BestEffort {
            inner.deliver_lossy(state.hash, &bytes);
            for (peer, conn, link) in readers {
                let len = 8 + bytes.len();
                if conn.max_datagram_size().is_some_and(|max| len <= max) {
                    let mut datagram = BytesMut::with_capacity(len);
                    datagram.put_u64_le(state.hash);
                    datagram.put_slice(&bytes);
                    let _ = conn.send_datagram(datagram.freeze());
                    continue;
                }
                if let Some(link) = link {
                    match link.lossy(state.hash, &mut shared) {
                        // Passed, or dropped for a subscriber that is behind.
                        Ok(()) => continue,
                        Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => continue,
                        Err(_) => {}
                    }
                }
                // Datagrams are not fragmented. A larger sample gets a stream
                // of its own. While the previous one is still going out it
                // waits, replacing any older sample that was waiting.
                let slot = Arc::clone(sending.streams.entry(peer).or_default());
                {
                    let mut slot = slot.lock().unwrap();
                    if slot.busy {
                        slot.next = Some((conn, bytes.clone()));
                        continue;
                    }
                    slot.busy = true;
                }
                let (hash, bytes, budget) = (state.hash, bytes.clone(), inner.timeout);
                tokio::spawn(async move {
                    let (mut conn, mut bytes) = (conn, bytes);
                    loop {
                        let _ = send_sample_stream(conn, hash, bytes, budget).await;
                        let mut slot = slot.lock().unwrap();
                        match slot.next.take() {
                            Some(next) => (conn, bytes) = next,
                            None => {
                                slot.busy = false;
                                break;
                            }
                        }
                    }
                });
            }
            return Ok(());
        }

        let ack = state.qos.reliability_mode() == Reliability::Reliable;
        let channels = &mut sending.channels;
        let delivery = async {
            let mut header = [0u8; 13];
            header[4] = u8::from(ack);
            header[5..].copy_from_slice(&state.hash.to_le_bytes());
            for (peer, conn, link) in &readers {
                if channels
                    .get(peer)
                    .is_none_or(|channel| channel.connection != conn.stable_id())
                {
                    let (send, recv) = conn.open_bi().await?;
                    let channel = Channel {
                        connection: conn.stable_id(),
                        send,
                        recv,
                    };
                    channels.insert(*peer, channel);
                }
                let channel = channels.get_mut(peer).unwrap();
                let token = match link {
                    Some(link) => link.ordered(state.hash, &mut shared).await.ok(),
                    None => None,
                };
                if let Some(token) = token {
                    header[..4].copy_from_slice(&SHARED.to_le_bytes());
                    channel.send.write_all(&header).await?;
                    channel.send.write_all(&token.to_le_bytes()).await?;
                } else {
                    header[..4].copy_from_slice(&(bytes.len() as u32).to_le_bytes());
                    channel.send.write_all(&header).await?;
                    channel.send.write_all(&bytes).await?;
                }
            }
            let mut accepted = inner.deliver_ordered(state.hash, &bytes, ack).await;
            if ack {
                for (peer, ..) in &readers {
                    let mut reply = [0u8; 1];
                    channels
                        .get_mut(peer)
                        .unwrap()
                        .recv
                        .read_exact(&mut reply)
                        .await?;
                    accepted &= reply[0] == 1;
                }
            }
            anyhow::Ok(accepted)
        };
        let failure = match tokio::time::timeout(inner.timeout, delivery).await {
            Ok(Ok(true)) => return Ok(()),
            Ok(Ok(false)) => anyhow!("a subscriber rejected the sample"),
            Ok(Err(error)) => error,
            Err(_) => anyhow!("pub/sub send timed out"),
        };
        // A stream may have stopped in the middle of a sample.
        sending.channels.clear();
        if ack {
            let message = format!("Reliable delivery failed: {failure:#}");
            inner.fail(&message).await;
            bail!(message);
        }
        Err(failure)
    }
}

#[derive(Debug)]
pub struct QuicSubscriber<T: Sendable> {
    node: Arc<QuicNode>,
    reader: String,
    hash: u64,
    chan: mpsc::Receiver<ArchivedMessage<T>>,
    stop: CancellationToken,
}
impl<T: Sendable> Drop for QuicSubscriber<T> {
    fn drop(&mut self) {
        self.stop.cancel();
        let inner = &self.node.inner;
        inner.undeclare(&self.reader);
        let mut state = inner.state.lock().unwrap();
        if let Some(sinks) = state.sinks.get_mut(&self.hash) {
            sinks.retain(|sink| sink.id != self.reader);
        }
    }
}

async fn enqueue<T: Sendable>(
    scratch: &mut AlignedVec,
    bytes: &[u8],
    qos: Qos,
    seen: &mut HashMap<String, u64>,
    sender: &mpsc::Sender<ArchivedMessage<T>>,
    stop: &CancellationToken,
) -> Result<()> {
    // The envelope is only read here, so its buffer is reused across samples.
    scratch.clear();
    scratch.extend_from_slice(bytes);
    let sample = rkyv::access::<ArchivedWireSample, rkyv::rancor::Error>(scratch)
        .context("Invalid pub/sub envelope")?;
    let sequence = sample.sequence.to_native();
    if qos.expects_reliable_delivery() && sample.qos == 0 {
        bail!("BestEffort publisher cannot satisfy a reliable subscriber");
    }
    if seen
        .get(sample.writer.as_str())
        .is_some_and(|last| *last >= sequence)
    {
        return Ok(());
    }
    let message = ArchivedMessage::from_aligned_bytes(aligned(
        sample.payload.len(),
        [sample.payload.as_slice()],
    ))?;
    if qos.reliability_mode() == Reliability::BestEffort {
        match sender.try_send(message) {
            Ok(()) | Err(mpsc::error::TrySendError::Full(_)) => {}
            Err(mpsc::error::TrySendError::Closed(_)) => bail!("subscriber closed"),
        }
    } else {
        tokio::select! {
            _ = stop.cancelled() => bail!("subscriber shut down"),
            result = sender.send(message) => result.map_err(|_| anyhow!("subscriber closed"))?,
        }
    }
    seen.insert(sample.writer.to_string(), sequence);
    Ok(())
}

impl<T: Sendable> QuicSubscriber<T> {
    pub(crate) async fn create(
        node: Arc<QuicNode>,
        topic: String,
        queue_size: usize,
        qos: Qos,
    ) -> Result<Self> {
        node.check_alive()?;
        ensure!(queue_size > 0, "subscriber queue size must be positive");
        ensure!(!topic.is_empty(), "topic must not be empty");
        let inner = &node.inner;
        let key = topic_key::<T>(inner.domain, &topic);
        if qos.expects_reliable_delivery() && inner.count(&key, false, Some(0)) > 0 {
            bail!("BestEffort publisher cannot satisfy a reliable subscriber on {topic}");
        }
        let reader = inner.identifier();
        let hash = key_hash(&key);
        let (tx, chan) = mpsc::channel(queue_size);
        let (sink, mut incoming) = mpsc::channel::<Incoming>(SINK_DEPTH);
        {
            // The sink is in place before peers learn of the reader.
            let mut state = inner.state.lock().unwrap();
            for (retained_key, sample) in state.retained.values() {
                if *retained_key == key
                    && let Some(bytes) = sample.lock().unwrap().clone()
                {
                    let _ = sink.try_send(Incoming { bytes, ack: None });
                }
            }
            state.sinks.entry(hash).or_default().push(Sink {
                id: reader.clone(),
                reliable: qos.expects_reliable_delivery(),
                tx: sink,
            });
        }
        inner.declare(
            &reader,
            Decl {
                reader: true,
                key,
                qos: qos_code(qos),
            },
        );
        let stop = node.shutdown.child_token();
        let quit = stop.clone();
        tokio::spawn(async move {
            let mut seen = HashMap::new();
            let mut scratch = AlignedVec::new();
            loop {
                tokio::select! {
                    _ = quit.cancelled() => break,
                    _ = tx.closed() => break,
                    received = incoming.recv() => {
                        let Some(Incoming { bytes, ack }) = received else { break; };
                        let result = enqueue(&mut scratch, &bytes, qos, &mut seen, &tx, &quit).await;
                        if let Some(ack) = ack {
                            let _ = ack.send(result.is_ok());
                        }
                        if let Err(error) = result {
                            log::warn!("Peer sample rejected: {error}");
                            break;
                        }
                    }
                }
            }
        });
        Ok(Self {
            node,
            reader,
            hash,
            chan,
            stop,
        })
    }
    pub(crate) fn try_next(&mut self) -> Option<T> {
        self.chan.try_recv().ok().map(|message| {
            message
                .deserialize()
                .expect("validated message must deserialize")
        })
    }
    pub(crate) async fn next_archived(&mut self) -> Option<ArchivedMessage<T>> {
        self.chan.recv().await
    }
}
