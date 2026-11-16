//! Large best-effort samples over QUIC: one unidirectional stream per sample.
//!
//! Datagrams are not fragmented, so a sample that does not fit one gets a
//! stream of its own. Per publisher and peer a limited number of such streams
//! is in flight ([`SamplePolicy::streams`], one by default); a sample
//! published while all are taken waits, and a newer one replaces it, so a slow
//! peer gets the latest sample next instead of a backlog.
//!
//! A sample that exceeds its budget is given up: its stream is reset, or, with
//! [`BudgetScope::Publication`], it is discarded while still waiting. The
//! budget bounds the sender's effort only. A reset cannot take back a sample
//! the peer has already received in full but not yet acknowledged, so a
//! subscriber can still see a sample older than the budget.

use bytes::Bytes;
use std::{
    io::Write,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::time::Instant;

/// What the budget of a large best-effort sample covers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BudgetScope {
    /// Writing the sample and its acknowledgement, from the moment its stream
    /// is open. Time spent waiting for an earlier sample is not counted.
    #[default]
    Stream,
    /// Everything from publication: waiting, opening the stream, writing and
    /// the acknowledgement.
    Publication,
}

/// How large best-effort samples are sent over QUIC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplePolicy {
    /// Time after which a sample is given up.
    pub budget: Duration,
    pub scope: BudgetScope,
    /// Sample streams in flight per publisher and peer, at least one.
    pub streams: usize,
}

/// Counters of the QUIC peer transport since the node started. All but
/// `outstanding` only grow.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct QuicStats {
    /// Large best-effort samples that went on to open a stream.
    pub started: u64,
    /// ... that the peer acknowledged in full.
    pub acked: u64,
    /// Payload bytes of the acknowledged ones.
    pub acked_bytes: u64,
    /// ... dropped while waiting, in favor of a newer sample.
    pub replaced: u64,
    /// ... dropped because their budget ran out before the stream was open.
    pub expired: u64,
    /// ... whose stream was reset because the budget ran out.
    pub reset: u64,
    /// ... whose stream failed: connection lost or peer stopped reading.
    pub failed: u64,
    /// ... whose stream could not be opened.
    pub open_failed: u64,
    /// Sample streams in flight right now.
    pub outstanding: u64,
    /// Sample streams received in full.
    pub received: u64,
    pub received_bytes: u64,
    /// Sample streams that ended early: reset by the sender or connection lost.
    pub incomplete: u64,
    /// Samples a subscriber dropped because it already had that sequence
    /// number or a newer one of the same writer: duplicates, and samples
    /// overtaken by a later one.
    pub stale: u64,
    /// UDP payload bytes of all peer connections, headers of QUIC included.
    pub udp_tx_bytes: u64,
    pub udp_rx_bytes: u64,
    pub sent_packets: u64,
    pub lost_packets: u64,
}

#[derive(Debug, Default)]
pub(crate) struct Counters {
    started: AtomicU64,
    acked: AtomicU64,
    acked_bytes: AtomicU64,
    replaced: AtomicU64,
    expired: AtomicU64,
    reset: AtomicU64,
    failed: AtomicU64,
    open_failed: AtomicU64,
    outstanding: AtomicU64,
    pub(crate) received: AtomicU64,
    pub(crate) received_bytes: AtomicU64,
    pub(crate) incomplete: AtomicU64,
    pub(crate) stale: AtomicU64,
}
pub(crate) fn bump(counter: &AtomicU64) {
    counter.fetch_add(1, Ordering::Relaxed);
}
impl Counters {
    pub(crate) fn snapshot(&self) -> QuicStats {
        let get = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        QuicStats {
            started: get(&self.started),
            acked: get(&self.acked),
            acked_bytes: get(&self.acked_bytes),
            replaced: get(&self.replaced),
            expired: get(&self.expired),
            reset: get(&self.reset),
            failed: get(&self.failed),
            open_failed: get(&self.open_failed),
            outstanding: get(&self.outstanding),
            received: get(&self.received),
            received_bytes: get(&self.received_bytes),
            incomplete: get(&self.incomplete),
            stale: get(&self.stale),
            ..QuicStats::default()
        }
    }
}

pub(crate) fn unix_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_nanos() as u64)
}

/// `MINOT_QUIC_TRACE=<file>` appends one JSON line per large best-effort
/// sample and peer: `tx` lines on the publishing node, `rx` lines on the
/// receiving one. Times are Unix nanoseconds, 0 where a step was not reached.
pub(crate) fn trace() -> Option<&'static Mutex<std::fs::File>> {
    static TRACE: OnceLock<Option<Mutex<std::fs::File>>> = OnceLock::new();
    TRACE
        .get_or_init(|| {
            let path = std::env::var_os("MINOT_QUIC_TRACE")?;
            let file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path);
            file.ok().map(Mutex::new)
        })
        .as_ref()
}
pub(crate) fn trace_line(line: std::fmt::Arguments) {
    if let Some(file) = trace() {
        let _ = writeln!(file.lock().unwrap(), "{line}");
    }
}

/// One large best-effort sample on its way to one peer.
#[derive(Debug)]
pub(crate) struct Sample {
    pub(crate) conn: quinn::Connection,
    pub(crate) peer: u64,
    pub(crate) hash: u64,
    pub(crate) bytes: Bytes,
    /// Of its writer; `u64::MAX` for a retained sample pushed to a late reader.
    pub(crate) sequence: u64,
    pub(crate) published: Instant,
    pub(crate) published_ns: u64,
    /// When it started to wait for a free stream, if it had to.
    waiting_ns: u64,
}
impl Sample {
    pub(crate) fn new(
        conn: quinn::Connection,
        peer: u64,
        hash: u64,
        bytes: Bytes,
        sequence: u64,
    ) -> Self {
        Self {
            conn,
            peer,
            hash,
            bytes,
            sequence,
            published: Instant::now(),
            published_ns: unix_ns(),
            waiting_ns: 0,
        }
    }
}

#[derive(Debug, Default)]
struct Steps {
    open_start: u64,
    open_done: u64,
    written: u64,
    /// Sender's view of the path when the stream opened, for the trace.
    path: Option<quinn::PathStats>,
    /// Smallest congestion window seen while the sample was on its stream.
    cwnd_min: u64,
}

/// The congestion state of a sample's connection as trace fields: window in
/// bytes and round-trip time in µs at the start (`0`) and end (`1`) of the
/// stream, smallest window in between, and what the stream's time added to
/// the connection's congestion events and lost and sent packets.
struct Path<'a>(&'a Sample, &'a Steps);
impl std::fmt::Display for Path<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let Some(start) = &self.1.path else {
            return Ok(());
        };
        let end = self.0.conn.stats().path;
        write!(
            f,
            ",\"cwnd0\":{},\"cwnd_min\":{},\"cwnd1\":{},\"rtt0\":{},\"rtt1\":{},\"min_rtt\":{},\
             \"cong\":{},\"lost\":{},\"sent\":{},\"mtu\":{}",
            start.cwnd,
            self.1.cwnd_min.min(end.cwnd),
            end.cwnd,
            start.rtt.as_micros(),
            end.rtt.as_micros(),
            end.min_rtt.as_micros(),
            end.congestion_events - start.congestion_events,
            end.lost_packets - start.lost_packets,
            end.sent_packets - start.sent_packets,
            end.current_mtu,
        )
    }
}

#[derive(Debug, Default)]
struct Slot {
    active: usize,
    next: Option<Sample>,
}

/// The sample streams of one publisher to one peer.
#[derive(Debug, Clone)]
pub(crate) struct SampleStreams {
    slot: Arc<Mutex<Slot>>,
    /// Identifier of this node, for the trace.
    node: u64,
    policy: SamplePolicy,
    counters: Arc<Counters>,
}
impl SampleStreams {
    pub(crate) fn new(node: u64, policy: SamplePolicy, counters: Arc<Counters>) -> Self {
        Self {
            slot: Arc::default(),
            node,
            policy,
            counters,
        }
    }
    /// Send `sample`, now if a stream is free, otherwise once one is. Only the
    /// newest waiting sample is kept.
    pub(crate) fn offer(&self, mut sample: Sample) {
        {
            let mut slot = self.slot.lock().unwrap();
            if slot.active >= self.policy.streams.max(1) {
                sample.waiting_ns = unix_ns();
                if let Some(old) = slot.next.replace(sample) {
                    bump(&self.counters.replaced);
                    self.trace(&old, &Steps::default(), "replaced");
                }
                return;
            }
            slot.active += 1;
        }
        let streams = self.clone();
        tokio::spawn(async move {
            loop {
                streams.send(sample).await;
                let mut slot = streams.slot.lock().unwrap();
                match slot.next.take() {
                    Some(next) => sample = next,
                    None => {
                        slot.active -= 1;
                        break;
                    }
                }
            }
        });
    }
    /// Streams in flight and whether a sample is waiting.
    #[cfg(test)]
    fn load(&self) -> (usize, bool) {
        let slot = self.slot.lock().unwrap();
        (slot.active, slot.next.is_some())
    }

    async fn send(&self, sample: Sample) {
        let counters = &self.counters;
        let from_publication = self.policy.scope == BudgetScope::Publication;
        let mut deadline = sample.published + self.policy.budget;
        let mut steps = Steps::default();
        if from_publication && Instant::now() >= deadline {
            bump(&counters.expired);
            self.trace(&sample, &steps, "expired");
            return;
        }
        bump(&counters.started);
        counters.outstanding.fetch_add(1, Ordering::Relaxed);
        let outcome = async {
            steps.open_start = unix_ns();
            let opened = if from_publication {
                match tokio::time::timeout_at(deadline, sample.conn.open_uni()).await {
                    Ok(opened) => opened,
                    Err(_) => {
                        bump(&counters.expired);
                        return "expired";
                    }
                }
            } else {
                sample.conn.open_uni().await
            };
            let Ok(mut stream) = opened else {
                bump(&counters.open_failed);
                return "open_failed";
            };
            steps.open_done = unix_ns();
            let tracing = trace().is_some();
            if tracing {
                let path = sample.conn.stats().path;
                steps.cwnd_min = path.cwnd;
                steps.path = Some(path);
            }
            if !from_publication {
                deadline = Instant::now() + self.policy.budget;
            }
            let written = &mut steps.written;
            let cwnd_min = &mut steps.cwnd_min;
            let sending = async {
                stream.write_all(&sample.hash.to_le_bytes()).await?;
                stream.write_chunk(sample.bytes.clone()).await?;
                stream.finish()?;
                *written = unix_ns();
                // Resolves once the peer has acknowledged the whole sample.
                anyhow::ensure!(stream.stopped().await?.is_none(), "peer stopped reading");
                anyhow::Ok(())
            };
            let sent = tokio::time::timeout_at(deadline, async {
                tokio::pin!(sending);
                loop {
                    tokio::select! {
                        done = &mut sending => break done,
                        _ = tokio::time::sleep(Duration::from_millis(20)), if tracing => {
                            *cwnd_min = (*cwnd_min).min(sample.conn.stats().path.cwnd);
                        }
                    }
                }
            })
            .await;
            match sent {
                Ok(Ok(())) => {
                    bump(&counters.acked);
                    let len = sample.bytes.len() as u64;
                    counters.acked_bytes.fetch_add(len, Ordering::Relaxed);
                    "acked"
                }
                Ok(Err(_)) => {
                    bump(&counters.failed);
                    "failed"
                }
                Err(_) => {
                    let _ = stream.reset(0u32.into());
                    bump(&counters.reset);
                    "reset"
                }
            }
        }
        .await;
        counters.outstanding.fetch_sub(1, Ordering::Relaxed);
        self.trace(&sample, &steps, outcome);
    }

    fn trace(&self, sample: &Sample, steps: &Steps, outcome: &str) {
        if trace().is_none() {
            return;
        }
        trace_line(format_args!(
            "{{\"ev\":\"tx\",\"node\":\"{:016x}\",\"peer\":\"{:016x}\",\"topic\":\"{:016x}\",\"seq\":{},\"bytes\":{},\
             \"published\":{},\"waiting\":{},\"open_start\":{},\"open_done\":{},\"written\":{},\
             \"end\":{},\"outcome\":\"{outcome}\"{}}}",
            self.node,
            sample.peer,
            sample.hash,
            sample.sequence as i64,
            sample.bytes.len(),
            sample.published_ns,
            sample.waiting_ns,
            steps.open_start,
            steps.open_done,
            steps.written,
            unix_ns(),
            Path(sample, steps),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, SocketAddr};

    /// Larger than the stream receive window, so a peer that does not read
    /// keeps the sample from completing.
    const STUCK: usize = 12 << 20;
    const SMALL: usize = 1 << 20;

    struct Pair {
        sender: quinn::Connection,
        receiver: quinn::Connection,
        _endpoints: (quinn::Endpoint, quinn::Endpoint),
    }
    /// Two connected endpoints with the transport settings of a node. The
    /// receiving side grants `uni` incoming streams.
    async fn pair(uni: u32) -> Pair {
        pair_with(uni, None).await
    }
    async fn pair_with(uni: u32, initial_window: Option<u64>) -> Pair {
        let (mut server, client) = crate::quic::quic_configs(false, initial_window).unwrap();
        let mut transport = crate::quic::transport_config(initial_window).unwrap();
        transport.max_concurrent_uni_streams(uni.into());
        server.transport_config(Arc::new(transport));
        let any = SocketAddr::from((Ipv4Addr::LOCALHOST, 0));
        let listener = quinn::Endpoint::server(server, any).unwrap();
        let mut dialer = quinn::Endpoint::client(any).unwrap();
        dialer.set_default_client_config(client);
        let address = listener.local_addr().unwrap();
        let (sender, receiver) = tokio::join!(
            async { dialer.connect(address, "minot").unwrap().await.unwrap() },
            async { listener.accept().await.unwrap().await.unwrap() },
        );
        Pair {
            sender,
            receiver,
            _endpoints: (listener, dialer),
        }
    }
    fn streams(budget_ms: u64, scope: BudgetScope, streams: usize) -> SampleStreams {
        let policy = SamplePolicy {
            budget: Duration::from_millis(budget_ms),
            scope,
            streams,
        };
        SampleStreams::new(0, policy, Arc::default())
    }
    fn sample(pair: &Pair, sequence: u64, len: usize) -> Sample {
        let bytes = Bytes::from(vec![sequence as u8; len]);
        Sample::new(pair.sender.clone(), 0, 7, bytes, sequence)
    }
    async fn until(what: &str, condition: impl Fn() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !condition() {
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    /// The sequence number of the next sample stream, or `None` if it was reset.
    async fn receive(conn: &quinn::Connection) -> Option<u8> {
        let mut stream = conn.accept_uni().await.unwrap();
        let bytes = stream.read_to_end(STUCK + 8).await.ok()?;
        Some(bytes[8])
    }

    #[tokio::test]
    async fn a_connection_starts_with_the_configured_window() {
        let default = pair(1).await.sender.stats().path.cwnd;
        let set = pair_with(1, Some(1_000_000)).await.sender.stats().path.cwnd;
        assert!(default < 100_000, "quinn's default window: {default}");
        assert!(set >= 1_000_000, "configured window: {set}");
    }

    #[tokio::test]
    async fn outstanding_streams_stay_within_the_limit() {
        for limit in [1, 2] {
            let pair = pair(64).await;
            let streams = streams(60_000, BudgetScope::Stream, limit);
            for sequence in 0..5 {
                streams.offer(sample(&pair, sequence, STUCK));
            }
            let stats = || streams.counters.snapshot();
            until("streams to start", || stats().started == limit as u64).await;
            tokio::time::sleep(Duration::from_millis(200)).await;
            assert_eq!(stats().started, limit as u64);
            assert_eq!(stats().outstanding, limit as u64);
            assert_eq!(streams.load(), (limit, true));
            // Reading frees the streams: the first ones and the newest follow.
            let mut received = Vec::new();
            for _ in 0..limit + 1 {
                received.push(receive(&pair.receiver).await.unwrap());
            }
            let mut expected: Vec<u8> = (0..limit as u8).collect();
            expected.push(4);
            assert_eq!(received, expected);
            until("streams to finish", || streams.load() == (0, false)).await;
            assert_eq!(stats().acked, limit as u64 + 1);
            assert_eq!(stats().outstanding, 0);
        }
    }

    #[tokio::test]
    async fn a_newer_sample_replaces_the_waiting_one() {
        let pair = pair(64).await;
        let streams = streams(60_000, BudgetScope::Stream, 1);
        for sequence in 0..4 {
            streams.offer(sample(&pair, sequence, STUCK));
        }
        let stats = streams.counters.snapshot();
        assert_eq!((stats.replaced, stats.expired), (2, 0));
        assert_eq!(receive(&pair.receiver).await, Some(0));
        assert_eq!(receive(&pair.receiver).await, Some(3));
        until("streams to finish", || streams.load() == (0, false)).await;
        assert_eq!(streams.counters.snapshot().started, 2);
    }

    #[tokio::test]
    async fn a_waiting_sample_expires_with_a_budget_from_publication() {
        let pair = pair(64).await;
        let streams = streams(300, BudgetScope::Publication, 1);
        // Published first, so its budget is over once the stuck stream gives
        // up; published later it would have the difference left to start.
        let waiting = sample(&pair, 1, SMALL);
        streams.offer(sample(&pair, 0, STUCK));
        streams.offer(waiting);
        until("the stream to give up", || streams.load() == (0, false)).await;
        let stats = streams.counters.snapshot();
        // The first ran out of time on its stream, the second while waiting.
        assert_eq!((stats.started, stats.reset, stats.expired), (1, 1, 1));
        assert_eq!(receive(&pair.receiver).await, None);
        let next = tokio::time::timeout(Duration::from_millis(300), pair.receiver.accept_uni());
        assert!(next.await.is_err(), "the expired sample was sent");
    }

    #[tokio::test]
    async fn a_waiting_sample_keeps_its_budget_with_a_budget_per_stream() {
        let pair = pair(64).await;
        let streams = streams(300, BudgetScope::Stream, 1);
        streams.offer(sample(&pair, 0, STUCK));
        streams.offer(sample(&pair, 1, SMALL));
        until("the first to give up", || {
            streams.counters.snapshot().reset == 1
        })
        .await;
        assert_eq!(receive(&pair.receiver).await, None);
        assert_eq!(receive(&pair.receiver).await, Some(1));
        until("streams to finish", || streams.load() == (0, false)).await;
        let stats = streams.counters.snapshot();
        assert_eq!((stats.reset, stats.acked, stats.expired), (1, 1, 0));
    }

    #[tokio::test]
    async fn opening_a_stream_counts_against_a_budget_from_publication() {
        // The peer grants no stream, so opening one never finishes.
        let pair = pair(0).await;
        let streams = streams(300, BudgetScope::Publication, 1);
        let begin = Instant::now();
        streams.offer(sample(&pair, 0, SMALL));
        until("the sample to expire", || streams.load() == (0, false)).await;
        assert!(begin.elapsed() >= Duration::from_millis(300));
        let stats = streams.counters.snapshot();
        assert_eq!((stats.expired, stats.reset, stats.outstanding), (1, 0, 0));
    }

    #[tokio::test]
    async fn a_lost_connection_ends_all_streams() {
        let pair = pair(64).await;
        let streams = streams(60_000, BudgetScope::Stream, 2);
        for sequence in 0..3 {
            streams.offer(sample(&pair, sequence, STUCK));
        }
        until("streams to start", || {
            streams.counters.snapshot().started == 2
        })
        .await;
        pair.receiver.close(0u32.into(), b"");
        until("streams to end", || streams.load() == (0, false)).await;
        let stats = streams.counters.snapshot();
        assert_eq!(stats.outstanding, 0);
        assert_eq!(stats.acked + stats.reset, 0);
        // The two in flight failed; the waiting one found the connection gone.
        assert_eq!(stats.failed + stats.open_failed, 3);
    }
}
