//! Resumable, credit-controlled chunk streams over a Minot network.
//!
//! Minot's pub/sub carries whole messages between nodes that are up. This adds
//! the two things a bulk transfer over an unreliable link needs on top of that:
//!
//! - **Chunking.** A multi-gigabyte object is never one message.
//! - **A window.** The receiver limits sender lookahead and memory use.
//! - **Resume.** When the link drops and comes back, the transfer continues from
//!   the receiver's watermark.
//!
//! Connection generations restore publishers and subscribers after a coordinator
//! restart. This crate retains in-flight chunks and resumes their transfer.
//!
//! # The window is also the resume protocol
//!
//! There is deliberately no separate `Resume` message. The receiver periodically
//! publishes a [`Window`] carrying two numbers: everything it has received
//! contiguously (`ack_through`), and how far the sender may run ahead
//! (`grant_through`). The sender always sends from `ack_through + 1`.
//!
//! That single idempotent message is credit, acknowledgement, and resume at
//! once. A `Window` lost in the disconnect costs nothing because the next one
//! supersedes it, and a sender that reconnects needs no special case: it is
//! already doing exactly what a resume would ask for.
//!
//! # QoS
//!
//! Flows run at [`Qos::TryReliable`], never `Qos::Reliable`. A `Reliable` node
//! is fatal by contract — peers monitor it and its death torpedoes the run — so
//! a laptop pulling a dataset over bad WiFi must never be one.

use anyhow::{Context, Result, anyhow};
use log::{debug, warn};
use mt_pubsub::{Node, Publisher, Qos, Subscriber};
use rkyv::{Archive, Deserialize, Serialize};

/// One piece of the stream, as it travels.
#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct Chunk {
    /// Position in the stream, starting at 1. Zero means "nothing yet" and is
    /// the initial value of a watermark, so it is never a valid chunk number.
    pub seq: u64,
    /// Set on the final chunk. The receiver finishes immediately when it arrives.
    pub last: bool,
    pub payload: Vec<u8>,
}

/// The receiver's view of the stream: what it has, and what it will take.
///
/// Idempotent and self-superseding — losing one costs nothing.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy)]
pub struct Window {
    /// Highest sequence received with no gaps before it. The sender may forget
    /// everything up to and including this, and resumes from the next one.
    pub ack_through: u64,
    /// Highest sequence the sender is allowed to send. Never send past this.
    pub grant_through: u64,
    /// Set once the receiver has the final chunk, to release the sender.
    pub complete: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct FlowConfig {
    /// Bytes per chunk. Phase 0 measured MCAP chunk reads at a p95 of ~700 KiB,
    /// so 1 MiB means one bag chunk is usually one flow chunk.
    pub chunk_bytes: usize,
    /// How many chunks the receiver lets the sender run ahead. This is the
    /// latency/memory trade: at 200 ms RTT a window of 32 hides essentially all
    /// of the round-trip cost, at the price of holding that many chunks on each
    /// side.
    pub window_chunks: u64,
    /// How often the receiver republishes its window even when nothing changed.
    /// This is what makes a reconnect recover on its own.
    pub window_interval: std::time::Duration,
    /// How long a sender waits with no window at all before giving up.
    pub stall_timeout: std::time::Duration,
    /// How long to wait before resending chunks the receiver has not
    /// acknowledged. A window whose watermark has not moved is the only loss
    /// signal there is, so this is what actually repairs a gap.
    pub retransmit_after: std::time::Duration,
}

impl Default for FlowConfig {
    fn default() -> Self {
        Self {
            chunk_bytes: 1024 * 1024,
            window_chunks: 32,
            window_interval: std::time::Duration::from_millis(250),
            stall_timeout: std::time::Duration::from_secs(120),
            retransmit_after: std::time::Duration::from_millis(750),
        }
    }
}

impl FlowConfig {
    pub fn validate(&self) -> Result<()> {
        if self.chunk_bytes == 0 {
            anyhow::bail!("FlowConfig::chunk_bytes must be greater than zero");
        }
        if self.window_chunks == 0 {
            anyhow::bail!(
                "FlowConfig::window_chunks must be greater than zero, or the sender can never send"
            );
        }
        if self.window_interval.is_zero() {
            anyhow::bail!("FlowConfig::window_interval must be greater than zero");
        }
        if self.retransmit_after.is_zero() {
            anyhow::bail!("FlowConfig::retransmit_after must be greater than zero");
        }
        Ok(())
    }
}

fn data_topic(flow: &str) -> String {
    format!("/_mt_flow/{flow}/data")
}

fn window_topic(flow: &str) -> String {
    format!("/_mt_flow/{flow}/window")
}

/// Publish, treating a failure while the link is down as "not yet".
///
/// A flow exists to survive disconnects, so the errors a disconnect produces
/// are the normal case, not the exceptional one. A dropped chunk stays in the
/// retain buffer and the repair timer sends it again once the link is back.
async fn publish_tolerantly<T: mt_sea::Sendable>(
    publisher: &Publisher<T>,
    value: &T,
    connection: &mt_sea::ConnectionState,
    what: &str,
) -> Result<()> {
    match publisher.publish(value).await {
        Ok(()) => Ok(()),
        Err(error) => {
            if connection.is_link_proven() && connection.is_connected() {
                Err(error).with_context(|| format!("flow: could not publish {what}"))
            } else {
                debug!("flow: {what} deferred while the link is down: {error}");
                Ok(())
            }
        }
    }
}

/// The sender's bookkeeping: what the receiver has confirmed, how far the
/// sender may run, and which chunks are still replayable.
///
/// Split out from [`FlowSender`] with no transport in it, because this is where
/// the subtle rules live — a stale window must never move a watermark backwards,
/// and a resume must never be promised for a chunk that has already been
/// dropped. Direct tests cover these transitions precisely.
#[derive(Debug)]
struct SenderWindow {
    /// Chunks sent but not yet acknowledged, oldest first. This is the replay
    /// buffer used by resume. It is bounded by the window because the sender
    /// never runs further ahead than the receiver allowed.
    retained: std::collections::VecDeque<Chunk>,
    /// Highest sequence handed to the wire.
    next_seq: u64,
    ack_through: u64,
    grant_through: u64,
    complete: bool,
}

impl SenderWindow {
    fn new() -> Self {
        Self {
            retained: std::collections::VecDeque::new(),
            next_seq: 0,
            ack_through: 0,
            grant_through: 0,
            complete: false,
        }
    }

    /// Fold in a window from the receiver.
    ///
    /// Windows can arrive out of order, especially just after a reconnect, so
    /// the watermarks only ever move forwards.
    fn apply(&mut self, window: Window) {
        if window.ack_through > self.ack_through {
            self.ack_through = window.ack_through;
        }
        if window.grant_through > self.grant_through {
            self.grant_through = window.grant_through;
        }
        self.complete |= window.complete;
        // Anything acknowledged can never need replaying again.
        while self
            .retained
            .front()
            .is_some_and(|chunk| chunk.seq <= self.ack_through)
        {
            self.retained.pop_front();
        }
    }

    fn record(&mut self, chunk: Chunk) {
        self.next_seq = chunk.seq;
        self.retained.push_back(chunk);
    }

    fn may_send(&self, seq: u64) -> bool {
        seq <= self.grant_through
    }

    /// Whether the receiver is still missing something already sent.
    fn has_unacked(&self) -> bool {
        !self.retained.is_empty()
    }

    /// Where a resume must restart from.
    fn resume_from(&self) -> u64 {
        self.ack_through + 1
    }

    /// The chunks that have to go out again before anything new does.
    fn replay(&self) -> Vec<Chunk> {
        let from = self.resume_from();
        self.retained
            .iter()
            .filter(|chunk| chunk.seq >= from)
            .cloned()
            .collect()
    }

    /// Whether the receiver is asking for something already dropped, which is
    /// the one situation a flow cannot recover from.
    fn resume_is_possible(&self) -> bool {
        let from = self.resume_from();
        // Nothing sent yet, or nothing retained: starting fresh is fine.
        match self.retained.front() {
            None => from == self.next_seq + 1,
            Some(oldest) => from >= oldest.seq,
        }
    }
}

/// The sending half of a flow.
pub struct FlowSender {
    chunks: Publisher<Chunk>,
    windows: Subscriber<Window>,
    config: FlowConfig,
    window: SenderWindow,
    connection: std::sync::Arc<mt_sea::ConnectionState>,
}

impl FlowSender {
    pub async fn open(node: &Node, flow: &str, config: FlowConfig) -> Result<Self> {
        config.validate()?;
        // The window subscription is established before the data publisher, so
        // the first window cannot be missed while the sender is still starting.
        let windows = node
            .create_subscriber::<Window>(window_topic(flow), 64, Qos::TryReliable)
            .await
            .with_context(|| format!("flow '{flow}': could not subscribe to its window"))?;
        let chunks = node
            .create_publisher::<Chunk>(data_topic(flow), Qos::TryReliable)
            .await
            .with_context(|| format!("flow '{flow}': could not publish its data"))?;
        Ok(Self {
            chunks,
            windows,
            config,
            window: SenderWindow::new(),
            connection: node.connection(),
        })
    }

    /// Drain any windows the receiver has published, keeping the newest.
    ///
    /// Non-blocking: a sender that is not waiting for credit should not stall
    /// to check for it.
    fn absorb_pending_windows(&mut self) {
        while let Some(window) = self.windows.try_next() {
            self.window.apply(window);
        }
    }

    /// Send everything in `source`, repairing gaps and resuming from the
    /// receiver's watermark whenever the link drops and returns.
    ///
    /// `source` is asked for bytes by absolute offset because a resume can move
    /// the read position backwards — the receiver's watermark, not the sender's
    /// progress, decides where the stream is.
    ///
    /// The loop has one rule that is easy to get wrong: **a window whose
    /// watermark has not moved is a loss report.** It is the only one the
    /// protocol has. A sender that only ever waits for credit to send *new*
    /// chunks will deadlock the moment a single chunk is dropped — the receiver
    /// cannot advance past the gap, so the grant freezes, so the sender waits
    /// forever for credit while the one chunk that would unblock everything sits
    /// in its retain buffer. Retransmission has to be driven by the timer below,
    /// independently of whether there is credit for anything new.
    pub async fn send_all<S>(&mut self, source: &mut S) -> Result<u64>
    where
        S: ChunkSource,
    {
        let mut sent_bytes = 0u64;
        let mut source_exhausted = false;
        let mut last_replay = tokio::time::Instant::now();
        let mut last_progress = tokio::time::Instant::now();
        let mut best_ack = 0u64;

        loop {
            self.absorb_pending_windows();

            if self.window.complete {
                return Ok(sent_bytes);
            }

            if self.window.ack_through > best_ack {
                best_ack = self.window.ack_through;
                last_progress = tokio::time::Instant::now();
            }

            if !self.window.resume_is_possible() {
                return Err(anyhow!(
                    "flow cannot resume: the receiver asked for chunk {}, which is \
                     older than anything still retained",
                    self.window.resume_from()
                ));
            }

            // 1. Repair. Anything the receiver has not acknowledged goes again,
            //    oldest first, within the credit it granted.
            if self.window.has_unacked() && last_replay.elapsed() >= self.config.retransmit_after {
                for chunk in self.window.replay() {
                    if !self.window.may_send(chunk.seq) {
                        break;
                    }
                    debug!("flow: retransmitting chunk {}", chunk.seq);
                    publish_tolerantly(&self.chunks, &chunk, &self.connection, "a retransmission")
                        .await?;
                }
                last_replay = tokio::time::Instant::now();
            }

            // 2. Send new data while there is both data and credit.
            if !source_exhausted {
                let seq = self.window.next_seq + 1;
                if self.window.may_send(seq) {
                    let offset = source.offset_of(seq, self.config.chunk_bytes);
                    let payload = source.read_at(offset, self.config.chunk_bytes)?;
                    let reached_end = payload.len() < self.config.chunk_bytes
                        || source.is_final(offset, &payload);
                    if payload.is_empty() && seq > 1 {
                        // The previous chunk already closed the stream.
                        source_exhausted = true;
                        continue;
                    }
                    sent_bytes += payload.len() as u64;
                    let chunk = Chunk {
                        seq,
                        last: reached_end,
                        payload,
                    };
                    source_exhausted = reached_end;
                    // Recorded before it is sent: if the publish is lost to a
                    // disconnect the chunk is already retained, so the repair
                    // step sends it again.
                    self.window.record(chunk.clone());
                    publish_tolerantly(&self.chunks, &chunk, &self.connection, "a chunk").await?;
                    last_progress = tokio::time::Instant::now();
                    continue;
                }
            }

            // 3. Nothing to send and nothing to repair yet: wait for the
            //    receiver to say something, but never longer than it would take
            //    to owe a retransmission.
            let idle = last_progress.elapsed();
            if idle >= self.config.stall_timeout {
                return Err(anyhow!(
                    "flow stalled: the receiver has not moved past chunk {} for {:?}",
                    self.window.ack_through,
                    self.config.stall_timeout
                ));
            }
            let patience = self
                .config
                .retransmit_after
                .min(self.config.stall_timeout - idle);
            match tokio::time::timeout(patience, self.windows.next()).await {
                Ok(Some(window)) => self.window.apply(window),
                Ok(None) => {
                    // The subscription ended. With reconnect enabled this means
                    // the node is really gone, not merely disconnected.
                    return Err(anyhow!("flow: the receiver's window channel closed"));
                }
                // Not an error: falling through re-runs the repair step, which
                // is exactly what a silent receiver needs.
                Err(_) => {}
            }
        }
    }
}

/// Where a [`FlowSender`] gets its bytes.
///
/// Reads use absolute offsets so resume can return to the receiver's watermark.
pub trait ChunkSource {
    fn read_at(&mut self, offset: u64, len: usize) -> Result<Vec<u8>>;

    /// Byte offset of chunk `seq`. The default is fixed-size chunking.
    fn offset_of(&self, seq: u64, chunk_bytes: usize) -> u64 {
        (seq - 1) * chunk_bytes as u64
    }

    /// Whether this read reached the end of the source.
    fn is_final(&self, _offset: u64, _payload: &[u8]) -> bool {
        false
    }
}

/// A flow sourced from a byte slice held in memory.
pub struct BytesSource {
    bytes: Vec<u8>,
}

impl BytesSource {
    pub fn new(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }
}

impl ChunkSource for BytesSource {
    fn read_at(&mut self, offset: u64, len: usize) -> Result<Vec<u8>> {
        let offset = offset as usize;
        if offset >= self.bytes.len() {
            return Ok(Vec::new());
        }
        let end = (offset + len).min(self.bytes.len());
        Ok(self.bytes[offset..end].to_vec())
    }

    fn is_final(&self, offset: u64, payload: &[u8]) -> bool {
        offset as usize + payload.len() >= self.bytes.len()
    }
}

/// The receiving half of a flow.
pub struct FlowReceiver {
    chunks: Subscriber<Chunk>,
    windows: Publisher<Window>,
    config: FlowConfig,
    connection: std::sync::Arc<mt_sea::ConnectionState>,
    /// Chunks received ahead of the watermark, waiting for the gap to fill.
    pending: std::collections::BTreeMap<u64, Chunk>,
    ack_through: u64,
    /// Sequence of the chunk marked `last`, once seen.
    final_seq: Option<u64>,
    complete: bool,
}

impl FlowReceiver {
    pub async fn open(node: &Node, flow: &str, config: FlowConfig) -> Result<Self> {
        config.validate()?;
        let chunks = node
            .create_subscriber::<Chunk>(
                data_topic(flow),
                config.window_chunks as usize * 2,
                Qos::TryReliable,
            )
            .await
            .with_context(|| format!("flow '{flow}': could not subscribe to its data"))?;
        let windows = node
            .create_publisher::<Window>(window_topic(flow), Qos::TryReliable)
            .await
            .with_context(|| format!("flow '{flow}': could not publish its window"))?;
        Ok(Self {
            chunks,
            windows,
            config,
            connection: node.connection(),
            pending: std::collections::BTreeMap::new(),
            ack_through: 0,
            final_seq: None,
            complete: false,
        })
    }

    fn window(&self) -> Window {
        Window {
            ack_through: self.ack_through,
            grant_through: self.ack_through + self.config.window_chunks,
            complete: self.complete,
        }
    }

    /// Receive the whole stream into `sink`.
    ///
    /// The window is republished on a timer as well as on progress, which is
    /// what lets a transfer recover from a disconnect without either side
    /// having to detect one: the sender simply hears an older watermark than it
    /// expected and replays from there.
    pub async fn receive_all<K>(&mut self, sink: &mut K) -> Result<u64>
    where
        K: ChunkSink,
    {
        let mut received_bytes = 0u64;
        let mut ticker = tokio::time::interval(self.config.window_interval);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        // Announce the opening credit before waiting for anything, or both
        // sides wait for each other.
        publish_tolerantly(
            &self.windows,
            &self.window(),
            &self.connection,
            "the opening window",
        )
        .await?;

        // Reset whenever the stream moves, so a long but healthy transfer is
        // never mistaken for a stalled one.
        let mut last_progress = tokio::time::Instant::now();
        loop {
            tokio::select! {
                _ = ticker.tick() => {
                    // Also the reconnect repair: an unchanged watermark tells a
                    // sender that came back where to resume from.
                    let window = self.window();
                    publish_tolerantly(&self.windows, &window, &self.connection, "a window")
                        .await?;
                    if last_progress.elapsed() >= self.config.stall_timeout && !self.complete {
                        return Err(anyhow!(
                            "flow stalled: no chunk past {} for {:?}",
                            self.ack_through,
                            self.config.stall_timeout
                        ));
                    }
                }
                chunk = self.chunks.next() => {
                    let Some(chunk) = chunk else {
                        return Err(anyhow!("flow: the sender's data channel closed"));
                    };
                    last_progress = tokio::time::Instant::now();
                    if chunk.seq <= self.ack_through {
                        // A replay of something already written. Expected after
                        // a reconnect. The sender cannot know what landed.
                        continue;
                    }
                    if chunk.last {
                        self.final_seq = Some(chunk.seq);
                    }
                    self.pending.insert(chunk.seq, chunk);

                    // Write out everything now contiguous.
                    while let Some(next) = self.pending.remove(&(self.ack_through + 1)) {
                        received_bytes += next.payload.len() as u64;
                        sink.write_at(
                            (next.seq - 1) * self.config.chunk_bytes as u64,
                            &next.payload,
                        )?;
                        self.ack_through = next.seq;
                    }

                    if self.final_seq == Some(self.ack_through) {
                        self.complete = true;
                        // Tell the sender it can stop. This is repeated because
                        // the message whose loss would hang the sender.
                        let window = self.window();
                        for _ in 0..3 {
                            publish_tolerantly(
                                &self.windows,
                                &window,
                                &self.connection,
                                "the completion window",
                            )
                            .await?;
                        }
                        return Ok(received_bytes);
                    }

                    let window = self.window();
                    publish_tolerantly(&self.windows, &window, &self.connection, "a window")
                        .await?;
                }
            }
        }
    }
}

/// Where a [`FlowReceiver`] puts the bytes.
///
/// Writes carry an absolute offset so that a sink can be a sparse file and a
/// resumed transfer lands in the right place.
pub trait ChunkSink {
    fn write_at(&mut self, offset: u64, payload: &[u8]) -> Result<()>;
}

/// A sink that collects the stream in memory.
#[derive(Default)]
pub struct BytesSink {
    bytes: Vec<u8>,
}

impl BytesSink {
    pub fn into_inner(self) -> Vec<u8> {
        self.bytes
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes
    }
}

impl ChunkSink for BytesSink {
    fn write_at(&mut self, offset: u64, payload: &[u8]) -> Result<()> {
        let offset = offset as usize;
        let end = offset + payload.len();
        if self.bytes.len() < end {
            self.bytes.resize(end, 0);
        }
        self.bytes[offset..end].copy_from_slice(payload);
        Ok(())
    }
}

/// Run both halves of a flow against a node pair.
///
/// Convenience for the common case where one process owns both ends, such as a
/// test or a local mirror.
pub async fn transfer<S, K>(
    sender_node: &Node,
    receiver_node: &Node,
    flow: &str,
    config: FlowConfig,
    source: &mut S,
    sink: &mut K,
) -> Result<u64>
where
    S: ChunkSource + Send,
    K: ChunkSink + Send,
{
    let mut receiver = FlowReceiver::open(receiver_node, flow, config).await?;
    let mut sender = FlowSender::open(sender_node, flow, config).await?;
    let (sent, received) = tokio::join!(sender.send_all(source), receiver.receive_all(sink));
    let sent = sent?;
    let received = received?;
    if sent != received {
        warn!("flow '{flow}': sent {sent} bytes but receiver wrote {received}");
    }
    Ok(received)
}

/// Re-exported so callers can configure the node a flow runs on.
pub use mt_pubsub::{CoordMode, NodeConfig, NodeOptions, ReconnectPolicy, Timing};

/// A flow's node should normally be built with this: WAN timing, reconnection
/// enabled, and a QoS whose death is not fatal to anyone else.
pub fn flow_node_config(name: impl Into<String>) -> NodeConfig {
    NodeConfig::new(name).mode(Qos::TryReliable).wan()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(seq: u64) -> Chunk {
        Chunk {
            seq,
            last: false,
            payload: vec![0u8; 4],
        }
    }

    fn window(ack_through: u64, grant_through: u64) -> Window {
        Window {
            ack_through,
            grant_through,
            complete: false,
        }
    }

    #[test]
    fn a_stale_window_never_moves_the_watermarks_backwards() {
        let mut state = SenderWindow::new();
        state.apply(window(10, 42));
        // Reordered or duplicated after a reconnect.
        state.apply(window(4, 8));
        assert_eq!(state.ack_through, 10);
        assert_eq!(state.grant_through, 42);
    }

    #[test]
    fn acknowledged_chunks_stop_being_retained() {
        let mut state = SenderWindow::new();
        for seq in 1..=5 {
            state.record(chunk(seq));
        }
        state.apply(window(3, 20));
        assert_eq!(
            state.retained.iter().map(|c| c.seq).collect::<Vec<_>>(),
            vec![4, 5],
            "only unacknowledged chunks are worth keeping"
        );
    }

    #[test]
    fn a_resume_replays_exactly_the_unacknowledged_tail() {
        let mut state = SenderWindow::new();
        for seq in 1..=5 {
            state.record(chunk(seq));
        }
        // The receiver only ever got as far as 2.
        state.apply(window(2, 20));
        assert_eq!(state.resume_from(), 3);
        assert_eq!(
            state.replay().iter().map(|c| c.seq).collect::<Vec<_>>(),
            vec![3, 4, 5],
            "everything past the receiver's watermark must go again, in order"
        );
    }

    #[test]
    fn credit_is_the_send_gate() {
        let mut state = SenderWindow::new();
        assert!(!state.may_send(1), "nothing may be sent before a grant");
        state.apply(window(0, 2));
        assert!(state.may_send(1));
        assert!(state.may_send(2));
        assert!(!state.may_send(3), "the sender must not outrun the window");
    }

    #[test]
    fn a_fresh_flow_can_always_start() {
        let state = SenderWindow::new();
        assert!(state.resume_is_possible());
        assert!(state.replay().is_empty());
    }

    #[test]
    fn resume_is_impossible_once_the_needed_chunk_was_dropped() {
        let mut state = SenderWindow::new();
        for seq in 1..=5 {
            state.record(chunk(seq));
        }
        state.apply(window(5, 20));
        assert!(state.retained.is_empty());
        // Everything is acknowledged, so resuming means starting at 6: fine.
        assert!(state.resume_is_possible());

        // Now a window arrives claiming *less* progress than a previous one.
        // The watermark refuses to move backwards, which is exactly what keeps
        // this recoverable while the bytes remain available.
        state.apply(window(2, 20));
        assert_eq!(state.ack_through, 5);
        assert!(state.resume_is_possible());
    }

    #[test]
    fn completion_latches() {
        let mut state = SenderWindow::new();
        state.apply(Window {
            ack_through: 3,
            grant_through: 9,
            complete: true,
        });
        state.apply(window(3, 9));
        assert!(
            state.complete,
            "completion must not be undone by a stale window"
        );
    }

    #[test]
    fn config_rejects_settings_that_could_never_transfer() {
        assert!(
            FlowConfig {
                window_chunks: 0,
                ..FlowConfig::default()
            }
            .validate()
            .is_err(),
            "a zero window can never let the sender send"
        );
        assert!(
            FlowConfig {
                chunk_bytes: 0,
                ..FlowConfig::default()
            }
            .validate()
            .is_err()
        );
        assert!(FlowConfig::default().validate().is_ok());
    }

    #[test]
    fn a_memory_sink_reassembles_out_of_order_writes() {
        let mut sink = BytesSink::default();
        sink.write_at(4, b"cd").unwrap();
        sink.write_at(0, b"ab").unwrap();
        assert_eq!(sink.as_slice(), b"ab\0\0cd");
    }
}
