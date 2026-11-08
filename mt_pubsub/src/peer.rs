//! Native Zenoh matching, with Minot acknowledgements and retained samples.
use super::NodeConfig;
use anyhow::{Context, Result, anyhow, bail, ensure};
use mt_sea::{ArchivedMessage, Qos, Reliability, Sendable};
use rkyv::{
    Archive, Place, Serialize, SerializeUnsized,
    munge::munge,
    rancor::{Fallible, Source},
    ser::{Writer, WriterExt},
    string::{ArchivedString, StringResolver},
    util::AlignedVec,
    vec::{ArchivedVec, VecResolver},
};
use std::{
    collections::{HashMap, HashSet},
    marker::PhantomData,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
};
use tokio_util::sync::CancellationToken;
use zenoh::{
    bytes::ZBytes,
    query::{ConsolidationMode, QueryTarget},
    sample::SampleKind,
};

static NEXT_ID: AtomicU64 = AtomicU64::new(1);
fn identifier(session: &zenoh::Session) -> String {
    format!(
        "{}_{}",
        session.zid(),
        NEXT_ID.fetch_add(1, Ordering::Relaxed)
    )
}
fn qos_code(qos: Qos) -> u8 {
    match qos.reliability_mode() {
        Reliability::BestEffort => 0,
        Reliability::Reliable => 1,
        Reliability::TryReliable => 2,
    }
}
fn hex(value: &str) -> String {
    value.bytes().map(|byte| format!("{byte:02x}")).collect()
}
fn topic_key<T>(domain: u16, topic: &str) -> String {
    // A versioned namespace prevents coordinated data and incompatible Rust
    // message types from being accidentally interpreted as peer samples.
    format!(
        "minot/{domain}/pubsub/v1/topics/{}/{}",
        hex(std::any::type_name::<T>()),
        hex(topic)
    )
}
fn ze(error: impl std::fmt::Display) -> anyhow::Error {
    anyhow!("Zenoh: {error}")
}
#[derive(Clone, Copy)]
enum TokenQuery {
    // Discovery counts and QoS checks only describe peers observed so far.
    Observed,
    // Reliable delivery must not mistake an incomplete snapshot for no readers.
    Complete,
}
/// Copy `slices` into one buffer aligned for rkyv, allocated once.
fn aligned<'a>(len: usize, slices: impl IntoIterator<Item = &'a [u8]>) -> AlignedVec {
    let mut bytes = AlignedVec::with_capacity(len);
    for slice in slices {
        bytes.extend_from_slice(slice);
    }
    bytes
}

/// The wire envelope. `payload` is the rkyv archive of the message.
#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
struct WireSample {
    writer: String,
    sequence: u64,
    qos: u8,
    payload: Vec<u8>,
}

/// Serializes as a [`WireSample`] whose payload is the archive of `value`,
/// in one pass: the message is archived in place inside the envelope instead
/// of on its own and then copied in as a `Vec<u8>`.
struct Outgoing<'a, T> {
    writer: &'a str,
    sequence: u64,
    qos: u8,
    value: &'a T,
}
struct OutgoingResolver {
    writer: StringResolver,
    payload_pos: usize,
    payload_len: usize,
}
impl<T> Archive for Outgoing<'_, T> {
    type Archived = ArchivedWireSample;
    type Resolver = OutgoingResolver;
    fn resolve(&self, resolver: Self::Resolver, out: Place<Self::Archived>) {
        munge!(let ArchivedWireSample { writer, sequence, qos, payload } = out);
        ArchivedString::resolve_from_str(self.writer, resolver.writer, writer);
        self.sequence.resolve((), sequence);
        self.qos.resolve((), qos);
        ArchivedVec::<u8>::resolve_from_len(
            resolver.payload_len,
            VecResolver::from_pos(resolver.payload_pos),
            payload,
        );
    }
}
impl<T, S> Serialize<S> for Outgoing<'_, T>
where
    T: Serialize<S>,
    S: Fallible + Writer<S::Error> + ?Sized,
    S::Error: Source,
    str: SerializeUnsized<S>,
{
    fn serialize(&self, serializer: &mut S) -> Result<OutgoingResolver, S::Error> {
        let writer = ArchivedString::serialize_from_str(self.writer, serializer)?;
        // Readers copy the payload to the start of an `AlignedVec`. Starting it
        // at the same alignment here makes every padding decision, and so every
        // byte, the same as `rkyv::to_bytes(value)`.
        let payload_pos = serializer.align(AlignedVec::<16>::ALIGNMENT)?;
        self.value.serialize_unsized(serializer)?;
        Ok(OutgoingResolver {
            writer,
            payload_pos,
            payload_len: serializer.pos() - payload_pos,
        })
    }
}

/// Lends a serialized sample to Zenoh without copying it, so the buffer can be
/// taken back and reused once Zenoh has written it out.
struct SharedBuffer(Arc<AlignedVec>);
impl AsRef<[u8]> for SharedBuffer {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

#[derive(Debug)]
pub struct PeerNode {
    pub(crate) session: Arc<zenoh::Session>,
    pub(crate) mode: Qos,
    pub(crate) shutdown: CancellationToken,
    pub(crate) connection: Arc<mt_sea::ConnectionState>,
    domain: u16,
    timeout: std::time::Duration,
    _alive: zenoh::liveliness::LivelinessToken,
}
impl Drop for PeerNode {
    fn drop(&mut self) {
        self.shutdown.cancel();
    }
}
impl PeerNode {
    pub(crate) async fn create(config: &NodeConfig) -> Result<Arc<Self>> {
        config.node_options().timing.validate()?;
        let mut network = mt_sea::network::zenoh_config(mt_sea::network::NetworkRole::PubSubPeer);
        // BestEffort uses CongestionControl::Drop. With zenoh's defaults (2 x 64 KiB
        // batches per queue, 50 ms deadline for all fragments of a message) any
        // sample larger than what the link drains in 50 ms is dropped by the
        // sender before it reaches the wire: at 20 Mbit/s that is everything
        // above ~128 KiB. Give large samples room and time to go out, like a DDS
        // best-effort writer. `wait_before_drop` stays at 1 ms, so a dead link
        // (queue already full) still drops new samples right away.
        network
            .insert_json5("transport/link/tx/queue/size/data", "16")
            .map_err(ze)?;
        network
            .insert_json5(
                "transport/link/tx/queue/congestion_control/drop/max_wait_before_drop_fragments",
                "1000000",
            )
            .map_err(ze)?;
        if let Some(endpoint) = &config.listen_endpoint {
            network
                .insert_json5("listen/endpoints", &serde_json::to_string(&[endpoint])?)
                .map_err(ze)?;
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
            network
                .insert_json5("connect/endpoints", &serde_json::to_string(&endpoints)?)
                .map_err(ze)?;
            network
                .insert_json5("connect/exit_on_failure", "false")
                .map_err(ze)?;
            network
                .insert_json5("scouting/multicast/enabled", "false")
                .map_err(ze)?;
        }
        let session = Arc::new(
            tokio::task::spawn_blocking(move || {
                mt_sea::network::open_zenoh_session_with_config(network)
            })
            .await??,
        );
        let domain = mt_sea::net::get_domain_id();
        let shutdown = CancellationToken::new();
        let id = identifier(&session);
        let nodes = format!("minot/{domain}/pubsub/v1/nodes");
        let live = session
            .liveliness()
            .declare_subscriber(format!("{nodes}/*/*"))
            .history(true)
            .await
            .map_err(ze)?;
        let torpedo = session
            .declare_subscriber(format!("minot/{domain}/pubsub/v1/torpedo"))
            .await
            .map_err(ze)?;
        let alive = session
            .liveliness()
            .declare_token(format!("{nodes}/{id}/{}", qos_code(config.node_mode())))
            .await
            .map_err(ze)?;
        let connection = mt_sea::ConnectionState::for_session(shutdown.clone());
        let session_task = Arc::clone(&session);
        let stop = shutdown.clone();
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = stop.cancelled() => break,
                    event = live.recv_async() => match event {
                        Ok(sample) if sample.kind() == SampleKind::Delete && sample.key_expr().as_str().ends_with("/1") => {
                            log::warn!("Reliable pub/sub peer disappeared: {}", sample.key_expr());
                            stop.cancel(); break;
                        }
                        Ok(_) => {},
                        Err(_) => { stop.cancel(); break; }
                    },
                    _ = torpedo.recv_async() => { stop.cancel(); break; }
                }
            }
            let _ = session_task.close().await;
        });
        Ok(Arc::new(Self {
            session,
            mode: config.node_mode(),
            shutdown,
            connection,
            domain,
            timeout: config.node_options().timing.registration_timeout(),
            _alive: alive,
        }))
    }
    fn check_alive(&self) -> Result<()> {
        ensure!(!self.shutdown.is_cancelled(), "pub/sub node is shut down");
        Ok(())
    }
    async fn fail(&self, message: &str) {
        let _ = self
            .session
            .put(
                format!("minot/{}/pubsub/v1/torpedo", self.domain),
                message.to_owned(),
            )
            .await;
        self.shutdown.cancel();
    }
    async fn tokens(&self, selector: String, policy: TokenQuery) -> Result<HashSet<String>> {
        self.check_alive()?;
        let replies = self
            .session
            .liveliness()
            .get(selector)
            .timeout(self.timeout)
            .await
            .map_err(ze)?;
        let mut tokens = HashSet::new();
        while let Ok(reply) = replies.recv_async().await {
            let result = reply.result();
            match result.as_ref() {
                Ok(sample) => {
                    tokens.insert(sample.key_expr().to_string());
                }
                // A growing mesh may not finalize all discovery replies within
                // the budget. Keep the observations we have; later liveliness
                // events still enforce subscriber QoS compatibility.
                Err(error)
                    if matches!(policy, TokenQuery::Observed)
                        && error.payload().to_bytes().as_ref() == b"Timeout" => {}
                Err(error) => return Err(ze(error)),
            }
        }
        Ok(tokens)
    }
}

#[derive(Debug)]
struct PublisherState {
    node: Arc<PeerNode>,
    key: String,
    writer: String,
    qos: Qos,
    publisher: zenoh::pubsub::Publisher<'static>,
    _alive: zenoh::liveliness::LivelinessToken,
    _cache_alive: Option<zenoh::liveliness::LivelinessToken>,
    stop: CancellationToken,
    sequence: AtomicU64,
    /// Serializes concurrent calls through clones to keep per-writer ordering,
    /// and holds the send buffer for reuse.
    serial: tokio::sync::Mutex<Option<AlignedVec>>,
    retained: Arc<Mutex<Option<ZBytes>>>,
}
impl Drop for PublisherState {
    fn drop(&mut self) {
        self.stop.cancel();
    }
}
#[derive(Debug)]
pub struct PeerPublisher<T> {
    state: Arc<PublisherState>,
    _type: PhantomData<fn() -> T>,
}
impl<T> Clone for PeerPublisher<T> {
    fn clone(&self) -> Self {
        Self {
            state: Arc::clone(&self.state),
            _type: PhantomData,
        }
    }
}
impl<T: Sendable> PeerPublisher<T> {
    pub(crate) async fn create(node: Arc<PeerNode>, topic: String, qos: Qos) -> Result<Self> {
        node.check_alive()?;
        ensure!(!topic.is_empty(), "topic must not be empty");
        let key = topic_key::<T>(node.domain, &topic);
        if qos_code(qos) == 0
            && !node
                .tokens(format!("{key}/readers/*/1"), TokenQuery::Observed)
                .await?
                .is_empty()
        {
            bail!("BestEffort publisher cannot satisfy a Reliable subscriber on {topic}");
        }
        let writer = identifier(&node.session);
        let publisher = node
            .session
            .declare_publisher(format!("{key}/data"))
            .congestion_control(qos.congestion_control())
            .reliability(qos.reliability())
            .await
            .map_err(ze)?;
        let stop = node.shutdown.child_token();
        let retained = Arc::new(Mutex::new(None::<ZBytes>));
        let cache_alive = if qos.is_transient_local() {
            let cache_key = format!("{key}/cache/{writer}");
            let cache = node
                .session
                .declare_queryable(cache_key.clone())
                .await
                .map_err(ze)?;
            let last = Arc::clone(&retained);
            let quit = stop.clone();
            tokio::spawn(async move {
                loop {
                    tokio::select! {
                        _ = quit.cancelled() => break,
                        result = cache.recv_async() => {
                            let Ok(query) = result else { break; };
                            let bytes = last.lock().unwrap().clone();
                            if let Some(bytes) = bytes { let _ = query.reply(cache_key.clone(), bytes).await; }
                        }
                    }
                }
            });
            Some(
                node.session
                    .liveliness()
                    .declare_token(format!("{key}/caches/{writer}"))
                    .await
                    .map_err(ze)?,
            )
        } else {
            None
        };
        let alive = node
            .session
            .liveliness()
            .declare_token(format!("{key}/writers/{writer}/{}", qos_code(qos)))
            .await
            .map_err(ze)?;
        Ok(Self {
            state: Arc::new(PublisherState {
                node,
                key,
                writer,
                qos,
                publisher,
                _alive: alive,
                _cache_alive: cache_alive,
                stop,
                sequence: AtomicU64::new(0),
                serial: tokio::sync::Mutex::new(None),
                retained,
            }),
            _type: PhantomData,
        })
    }
    pub(crate) async fn subscriber_count(&self) -> Result<usize> {
        Ok(self
            .state
            .node
            .tokens(
                format!("{}/readers/*/*", self.state.key),
                TokenQuery::Observed,
            )
            .await?
            .len())
    }
    pub(crate) async fn publish(&self, value: &T) -> Result<()> {
        let state = &self.state;
        state.node.check_alive()?;
        let mut spare = state.serial.lock().await;
        let mut buffer = spare.take().unwrap_or_default();
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
        let bytes = ZBytes::from(bytes::Bytes::from_owner(SharedBuffer(buffer.clone())));
        let result = self.send(bytes).await;
        // Zenoh copies the sample into its batches while sending, so the buffer
        // is normally ours again. It is not while a retained copy or a local
        // subscriber still holds it; the next publish then allocates.
        if let Ok(buffer) = Arc::try_unwrap(buffer) {
            *spare = Some(buffer);
        }
        result
    }
    async fn send(&self, bytes: ZBytes) -> Result<()> {
        let state = &self.state;
        if state.qos.is_transient_local() {
            *state.retained.lock().unwrap() = Some(bytes.clone());
        }
        if state.qos.reliability_mode() != Reliability::Reliable {
            // BestEffort drops on congestion; TryReliable uses Zenoh's bounded
            // reliable enqueue and returns any send-budget failure to the caller.
            tokio::time::timeout(state.node.timeout, state.publisher.put(bytes))
                .await
                .context("pub/sub send timed out")?
                .map_err(ze)?;
            return Ok(());
        }
        let expected: HashSet<_> = state
            .node
            .tokens(format!("{}/readers/*/*", state.key), TokenQuery::Complete)
            .await?
            .into_iter()
            .map(|token| token.rsplit_once('/').unwrap().0.to_owned())
            .collect();
        if expected.is_empty() {
            return Ok(());
        }
        let replies = state
            .node
            .session
            .get(format!("{}/readers/*", state.key))
            .payload(bytes)
            .target(QueryTarget::All)
            .consolidation(ConsolidationMode::None)
            .timeout(state.node.timeout)
            .await
            .map_err(ze)?;
        let mut acknowledged = HashSet::new();
        while let Ok(reply) = replies.recv_async().await {
            if let Ok(sample) = reply.result() {
                acknowledged.insert(sample.key_expr().to_string());
            }
        }
        if !expected.is_subset(&acknowledged) {
            let message = format!(
                "Reliable delivery missing {} subscriber acknowledgements",
                expected.difference(&acknowledged).count()
            );
            state.node.fail(&message).await;
            bail!(message);
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct PeerSubscriber<T: Sendable> {
    _node: Arc<PeerNode>,
    chan: tokio::sync::mpsc::Receiver<ArchivedMessage<T>>,
    stop: CancellationToken,
}
impl<T: Sendable> Drop for PeerSubscriber<T> {
    fn drop(&mut self) {
        self.stop.cancel();
    }
}

async fn enqueue<T: Sendable>(
    scratch: &mut AlignedVec,
    bytes: &ZBytes,
    qos: Qos,
    seen: &mut HashMap<String, u64>,
    sender: &tokio::sync::mpsc::Sender<ArchivedMessage<T>>,
    stop: &CancellationToken,
) -> Result<()> {
    // The envelope is only read here, so its buffer is reused across samples.
    scratch.clear();
    scratch.reserve(bytes.len());
    for slice in bytes.slices() {
        scratch.extend_from_slice(slice);
    }
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
            Ok(()) | Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {}
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => bail!("subscriber closed"),
        }
    } else {
        tokio::select! {
            _ = stop.cancelled() => bail!("subscriber shut down"),
            result = sender.send(message) => result.map_err(ze)?,
        }
    }
    seen.insert(sample.writer.to_string(), sequence);
    Ok(())
}

impl<T: Sendable> PeerSubscriber<T> {
    pub(crate) async fn create(
        node: Arc<PeerNode>,
        topic: String,
        queue_size: usize,
        qos: Qos,
    ) -> Result<Self> {
        node.check_alive()?;
        ensure!(queue_size > 0, "subscriber queue size must be positive");
        ensure!(!topic.is_empty(), "topic must not be empty");
        let key = topic_key::<T>(node.domain, &topic);
        if qos.expects_reliable_delivery()
            && !node
                .tokens(format!("{key}/writers/*/0"), TokenQuery::Observed)
                .await?
                .is_empty()
        {
            bail!("BestEffort publisher cannot satisfy a reliable subscriber on {topic}");
        }
        let reader_key = format!("{key}/readers/{}", identifier(&node.session));
        let subscriber = node
            .session
            .declare_subscriber(format!("{key}/data"))
            .await
            .map_err(ze)?;
        let queryable = node
            .session
            .declare_queryable(reader_key.clone())
            .await
            .map_err(ze)?;
        let token = node
            .session
            .liveliness()
            .declare_token(format!("{reader_key}/{}", qos_code(qos)))
            .await
            .map_err(ze)?;
        let writers = node
            .session
            .liveliness()
            .declare_subscriber(format!("{key}/writers/*/*"))
            .history(true)
            .await
            .map_err(ze)?;
        // Only retained writers advertise caches. Querying every volatile
        // writer on discovery turns a swarm launch into an all-to-all storm.
        let caches = node
            .session
            .liveliness()
            .declare_subscriber(format!("{key}/caches/*"))
            .history(true)
            .await
            .map_err(ze)?;
        let (snapshot_tx, mut snapshot_rx) = tokio::sync::mpsc::channel::<ZBytes>(32);
        let session_for_cache = Arc::clone(&node.session);
        let cache_timeout = node.timeout;
        let (tx, chan) = tokio::sync::mpsc::channel(queue_size);
        let stop = node.shutdown.child_token();
        let quit = stop.clone();
        tokio::spawn(async move {
            let _token = token;
            let mut seen = HashMap::new();
            let mut scratch = AlignedVec::new();
            loop {
                tokio::select! {
                    _ = quit.cancelled() => break,
                    _ = tx.closed() => break,
                    result = writers.recv_async() => {
                        let Ok(sample) = result else { break; };
                        if sample.kind() == SampleKind::Put && qos.expects_reliable_delivery() && sample.key_expr().as_str().ends_with("/0") {
                            log::warn!("BestEffort publisher cannot satisfy a reliable subscriber"); break;
                        }
                        if sample.kind() == SampleKind::Delete {
                            if let Some(writer) = sample.key_expr().as_str().rsplit('/').nth(1) { seen.remove(writer); }
                        }
                    },
                    result = caches.recv_async() => {
                        let Ok(sample) = result else { break; };
                        if sample.kind() == SampleKind::Put {
                            // Cache availability is announced after declaring the
                            // queryable. History covers existing retained writers;
                            // new announcements cover writers discovered later.
                            // Sequence numbers deduplicate overlap with live data.
                            if let Some(writer) = sample.key_expr().as_str().rsplit('/').next() {
                                let cache_key = format!("{key}/cache/{writer}");
                                let session = Arc::clone(&session_for_cache);
                                let sender = snapshot_tx.clone();
                                let stop = quit.clone();
                                tokio::spawn(async move {
                                    let Ok(replies) = session.get(cache_key).target(QueryTarget::All)
                                        .consolidation(ConsolidationMode::None).timeout(cache_timeout).await else { return; };
                                    loop {
                                        tokio::select! {
                                            _ = stop.cancelled() => break,
                                            result = replies.recv_async() => {
                                                let Ok(reply) = result else { break; };
                                                if let Ok(sample) = reply.result() {
                                                    tokio::select! {
                                                        _ = stop.cancelled() => break,
                                                        result = sender.send(sample.payload().clone()) => if result.is_err() { break; },
                                                    }
                                                }
                                            }
                                        }
                                    }
                                });
                            }
                        }
                    },
                    Some(bytes) = snapshot_rx.recv() => {
                        if let Err(error) = enqueue(&mut scratch, &bytes, qos, &mut seen, &tx, &quit).await {
                            log::warn!("Discovered retained sample rejected: {error}"); break;
                        }
                    },
                    result = subscriber.recv_async() => {
                        let Ok(sample) = result else { break; };
                        if let Err(error) = enqueue(&mut scratch, sample.payload(), qos, &mut seen, &tx, &quit).await { log::warn!("Peer sample rejected: {error}"); break; }
                    },
                    result = queryable.recv_async() => {
                        let Ok(query) = result else { break; };
                        if let Some(bytes) = query.payload() {
                            match enqueue(&mut scratch, bytes, qos, &mut seen, &tx, &quit).await {
                                Ok(()) => { let _ = query.reply(reader_key.clone(), Vec::<u8>::new()).await; },
                                Err(error) => { let _ = query.reply_err(error.to_string()).await; }
                            }
                        }
                    }
                }
            }
        });
        Ok(Self {
            _node: node,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Debug, PartialEq)]
    struct Cloud {
        frame_id: String,
        width: u32,
        stamp: u64,
        fields: Vec<(String, u8)>,
        data: Vec<u8>,
    }

    #[test]
    fn one_pass_envelope_matches_nested_serialization() {
        let value = Cloud {
            frame_id: "drone_7/1234".into(),
            width: 100_000,
            stamp: 1_700_000_000_123,
            fields: vec![("x".into(), 7), ("intensity".into(), 7)],
            data: (0..1_600_003u32).map(|i| (i * 31) as u8).collect(),
        };
        let nested = rkyv::to_bytes::<rkyv::rancor::Error>(&value).unwrap();
        // Reused buffers start dirty and too small or too large.
        for start in [AlignedVec::<16>::new(), AlignedVec::with_capacity(4 << 20)] {
            for writer in ["w", "a_writer_name_longer_than_inline"] {
                let mut buffer = start.clone();
                buffer.extend_from_slice(b"stale");
                buffer.clear();
                let sample = Outgoing {
                    writer,
                    sequence: 42,
                    qos: 2,
                    value: &value,
                };
                let wire = rkyv::api::high::to_bytes_in::<_, rkyv::rancor::Error>(&sample, buffer)
                    .unwrap();
                // Old readers deserialize the envelope.
                let old = rkyv::from_bytes::<WireSample, rkyv::rancor::Error>(&wire).unwrap();
                assert_eq!(old.writer, writer);
                assert_eq!(old.sequence, 42);
                assert_eq!(old.qos, 2);
                assert_eq!(old.payload, nested.as_slice());
                let message = ArchivedMessage::<Cloud>::from_aligned_bytes(aligned(
                    old.payload.len(),
                    [old.payload.as_slice()],
                ))
                .unwrap();
                assert_eq!(message.deserialize().unwrap(), value);
            }
        }
        // New readers accept envelopes from old writers.
        let old = WireSample {
            writer: "old".into(),
            sequence: 1,
            qos: 0,
            payload: nested.to_vec(),
        };
        let wire = rkyv::to_bytes::<rkyv::rancor::Error>(&old).unwrap();
        let sample = rkyv::access::<ArchivedWireSample, rkyv::rancor::Error>(&wire).unwrap();
        assert_eq!(sample.payload.as_slice(), nested.as_slice());
    }
}
