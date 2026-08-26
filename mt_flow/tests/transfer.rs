//! A flow must deliver every byte, in order, across a link that breaks.
//!
//! The coordinator runs as a subprocess so it can be `SIGKILL`ed mid-transfer,
//! an embedded one shares the test process and cannot be killed the way a real
//! partition kills it.

use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mt_flow::{BytesSink, BytesSource, FlowConfig, FlowReceiver, FlowSender};
use mt_pubsub::{CoordMode, Node, NodeConfig, Qos, Timing};

/// Local-only Minot uses one fixed endpoint, so these must not overlap.
static COORDINATOR_PORT: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

fn test_timing() -> Timing {
    Timing {
        heartbeat_interval_ms: 200,
        heartbeat_suppress_ms: 100,
        disconnect_timeout_ms: 1_000,
        registration_timeout_ms: 5_000,
        peer_dead_threshold: 3,
    }
}

struct Coordinator {
    child: Child,
}

impl Coordinator {
    async fn start() -> Option<Self> {
        let binary = coordinator_binary()?;
        let child = Command::new(binary)
            .arg("coordinator")
            .arg("--local-only")
            .env("RUST_LOG", "off")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .ok()?;
        let coordinator = Self { child };
        if !wait_until(
            Duration::from_secs(30),
            mt_sea::network::local_router_is_running,
        )
        .await
        {
            return None;
        }
        Some(coordinator)
    }

    async fn kill(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        wait_until(Duration::from_secs(30), || {
            !mt_sea::network::local_router_is_running()
        })
        .await;
    }
}

impl Drop for Coordinator {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn coordinator_binary() -> Option<std::path::PathBuf> {
    let mut dir = std::env::current_exe().ok()?;
    dir.pop();
    if dir.ends_with("deps") {
        dir.pop();
    }
    let candidate = dir.join("minot");
    candidate.is_file().then_some(candidate)
}

async fn wait_until(limit: Duration, mut condition: impl FnMut() -> bool) -> bool {
    let deadline = tokio::time::Instant::now() + limit;
    while tokio::time::Instant::now() < deadline {
        if condition() {
            return true;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    condition()
}

fn unique(prefix: &str) -> String {
    format!(
        "{prefix}_{}_{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )
}

/// Deterministic, position-dependent bytes, so a mis-ordered or duplicated
/// chunk produces a deterministic mismatch.
fn payload(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i.wrapping_mul(31) % 251) as u8).collect()
}

async fn node(name: &str) -> Node {
    Node::create(
        NodeConfig::new(name.to_owned())
            .mode(Qos::TryReliable)
            .coord_mode(CoordMode::External)
            .timing(test_timing()),
    )
    .await
    .expect("node should register with the running coordinator")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_multi_chunk_payload_arrives_intact() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let sender_node = node(&unique("flow_tx")).await;
    let receiver_node = node(&unique("flow_rx")).await;

    let config = FlowConfig {
        chunk_bytes: 4096,
        window_chunks: 8,
        window_interval: Duration::from_millis(50),
        stall_timeout: Duration::from_secs(60),
        retransmit_after: Duration::from_millis(300),
    };
    // Deliberately not a whole number of chunks, so the final short chunk is
    // exercised across a partial final chunk.
    let expected = payload(4096 * 20 + 1234);
    let flow = unique("intact");

    let mut source = BytesSource::new(expected.clone());
    let mut sink = BytesSink::default();
    let received = mt_flow::transfer(
        &sender_node,
        &receiver_node,
        &flow,
        config,
        &mut source,
        &mut sink,
    )
    .await
    .expect("transfer should complete");

    assert_eq!(received, expected.len() as u64);
    assert_eq!(
        sink.as_slice(),
        expected.as_slice(),
        "every byte must arrive, in order"
    );

    coordinator.kill().await;
}

/// The headline: kill the coordinator mid-transfer and the flow finishes anyway,
/// with the payload byte-identical. Nothing is restarted by the caller, since the
/// nodes reconnect themselves and the flow resumes from the receiver's
/// watermark.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_transfer_survives_a_coordinator_restart() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let sender_node = Arc::new(node(&unique("resume_tx")).await);
    let receiver_node = Arc::new(node(&unique("resume_rx")).await);

    // Wait until both links are genuinely carrying traffic, so the kill below
    // interrupts a transfer with active data.
    for node in [&sender_node, &receiver_node] {
        assert!(
            wait_until(Duration::from_secs(30), || node
                .connection()
                .is_link_proven())
            .await,
            "both nodes should be talking to the coordinator before the test starts"
        );
    }

    let config = FlowConfig {
        chunk_bytes: 16 * 1024,
        window_chunks: 4,
        window_interval: Duration::from_millis(50),
        // Generous, because this rides out a coordinator that is gone
        // for seconds.
        stall_timeout: Duration::from_secs(90),
        retransmit_after: Duration::from_millis(300),
    };
    // Big enough that the transfer is still running when the coordinator dies.
    let expected = payload(16 * 1024 * 400);
    let flow = unique("resume");

    let mut receiver = FlowReceiver::open(&receiver_node, &flow, config)
        .await
        .expect("receiver should open");
    let mut sender = FlowSender::open(&sender_node, &flow, config)
        .await
        .expect("sender should open");

    let expected_for_send = expected.clone();
    let send = tokio::spawn(async move {
        let mut source = BytesSource::new(expected_for_send);
        sender.send_all(&mut source).await
    });
    let receive = tokio::spawn(async move {
        let mut sink = BytesSink::default();
        let received = receiver.receive_all(&mut sink).await?;
        Ok::<_, anyhow::Error>((received, sink.into_inner()))
    });

    // Let some of it through, then take the network away.
    tokio::time::sleep(Duration::from_millis(600)).await;
    coordinator.kill().await;
    tokio::time::sleep(Duration::from_secs(2)).await;
    let mut replacement = Coordinator::start()
        .await
        .expect("coordinator should restart");

    let sent = tokio::time::timeout(Duration::from_secs(120), send)
        .await
        .expect("sender should not hang")
        .expect("sender task should not panic")
        .expect("sender should finish despite the restart");
    let (received, bytes) = tokio::time::timeout(Duration::from_secs(120), receive)
        .await
        .expect("receiver should not hang")
        .expect("receiver task should not panic")
        .expect("receiver should finish despite the restart");

    assert_eq!(sent, expected.len() as u64, "sender should send everything");
    assert_eq!(received, expected.len() as u64);
    assert_eq!(
        bytes.len(),
        expected.len(),
        "the resumed stream must not be short or doubled"
    );
    assert!(
        bytes == expected,
        "every byte must survive the restart, in order and without duplication"
    );

    replacement.kill().await;
}
