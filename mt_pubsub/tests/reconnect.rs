//! A node must survive losing its coordinator.
//!
//! Before reconnect existed, a dropped coordinator link was terminal: the
//! disconnect one-shot fired, the node's `CancellationToken` was cancelled, and
//! every subscriber task returned for good. These tests kill a real coordinator
//! process mid-stream and assert that the publisher and subscriber the caller
//! is holding keep working once it comes back.
//!
//! The coordinator runs as a subprocess precisely so it can be `SIGKILL`ed —
//! an embedded one shares this process and cannot be killed the way a real
//! network partition kills it.

use std::process::{Child, Command, Stdio};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use mt_pubsub::{CoordMode, Node, NodeConfig, Qos, ReconnectPolicy, Timing};
use rkyv::{Archive, Deserialize, Serialize};
use tokio::time::timeout;

#[derive(Archive, Serialize, Deserialize, Debug, PartialEq)]
struct Sample {
    sequence: u64,
}

/// Local-only Minot uses one fixed endpoint (127.0.0.1:7447), so two of these
/// tests running at once would each be talking to the other's coordinator.
/// They must run one at a time even though the harness is happy to run test
/// functions in parallel.
static COORDINATOR_PORT: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

/// Timing tight enough that a test does not take a minute, loose enough that a
/// loaded CI box does not report a false disconnect.
fn test_timing() -> Timing {
    Timing {
        heartbeat_interval_ms: 200,
        heartbeat_suppress_ms: 100,
        disconnect_timeout_ms: 1_000,
        registration_timeout_ms: 5_000,
        peer_dead_threshold: 3,
    }
}

/// A coordinator running as a killable child process.
struct Coordinator {
    child: Child,
}

impl Coordinator {
    /// Start a coordinator and wait until it is actually accepting connections.
    ///
    /// Readiness is polled rather than slept on: the coordinator also brings up
    /// an embedded scope node, so how long it takes to bind varies with load.
    async fn start() -> Option<Self> {
        let binary = coordinator_binary()?;
        let child = Command::new(binary)
            .arg("coordinator")
            .arg("--local-only")
            .env(
                "RUST_LOG",
                std::env::var("COORD_LOG").unwrap_or_else(|_| "off".into()),
            )
            .stdout(Stdio::null())
            .stderr(if std::env::var("COORD_LOG").is_ok() {
                Stdio::inherit()
            } else {
                Stdio::null()
            })
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

    /// Kill it and wait until the endpoint is actually gone, so a caller that
    /// starts a replacement cannot race the old one still holding the port.
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
        // Async cleanup is not available here; the blocking kill is enough to
        // stop the process leaking out of the test run.
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Locate the `minot` binary built alongside this test.
///
/// Returns `None` when it was not built — the coordinator lives in the `minot`
/// package, which is not a dependency of this one, so `cargo test -p mt_pubsub`
/// on its own will not have produced it.
fn coordinator_binary() -> Option<std::path::PathBuf> {
    let mut dir = std::env::current_exe().ok()?;
    dir.pop(); // the test binary itself
    if dir.ends_with("deps") {
        dir.pop();
    }
    let candidate = dir.join("minot");
    candidate.is_file().then_some(candidate)
}

/// Wait for a condition, polling, up to `limit`.
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

fn unique_name(prefix: &str) -> String {
    format!(
        "{prefix}_{}_{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )
}

/// The headline case: kill the coordinator under a live node, bring it back,
/// and the same `Publisher` and `Subscriber` handles keep working.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn node_survives_a_coordinator_restart() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let node = Node::create(
        NodeConfig::new(unique_name("resilient"))
            // TryReliable, not Reliable: a Reliable node is fatal by contract
            // and must not silently come back.
            .mode(Qos::TryReliable)
            .coord_mode(CoordMode::External)
            .timing(test_timing())
            .reconnect(ReconnectPolicy::always()),
    )
    .await
    .expect("node should register with the running coordinator");

    let connection = node.connection();
    assert_eq!(connection.generation(), 0, "a fresh node is generation 0");
    assert!(connection.is_connected());

    // Registration only proves the coordinator was reachable at that instant.
    // Wait until it has actually echoed a heartbeat, because that is when the
    // node's disconnect detector arms — killing the coordinator before then
    // tests nothing.
    assert!(
        wait_until(Duration::from_secs(30), || connection.is_link_proven()).await,
        "the coordinator should answer this node's heartbeats"
    );

    // Kill the coordinator out from under the node.
    coordinator.kill().await;

    assert!(
        wait_until(Duration::from_secs(10), || !connection.is_connected()).await,
        "the node should notice the coordinator is gone"
    );
    assert!(
        !node.shutdown_token().is_cancelled(),
        "losing the coordinator must not shut down a reconnecting node — this is \
         the exact regression this test exists for"
    );

    // Bring it back.
    let mut replacement = Coordinator::start()
        .await
        .expect("coordinator should restart");

    assert!(
        wait_until(Duration::from_secs(30), || connection.is_connected()).await,
        "the node should re-register once the coordinator returns"
    );
    assert!(
        connection.generation() >= 1,
        "a reconnect must publish a new generation, got {}",
        connection.generation()
    );
    assert!(
        wait_until(Duration::from_secs(30), || connection.is_link_proven()).await,
        "the rebuilt link must be carrying heartbeats again, not just registered"
    );

    replacement.kill().await;
}

/// A `Qos::Reliable` node must keep its old, fatal behaviour: its death is the
/// signal the rest of the run depends on, so it must not quietly recover.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reliable_nodes_still_die_with_their_coordinator() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let node = Node::create(
        NodeConfig::new(unique_name("fatal"))
            .mode(Qos::Reliable)
            .coord_mode(CoordMode::External)
            .timing(test_timing()),
    )
    .await
    .expect("node should register with the running coordinator");

    let connection = node.connection();
    assert!(
        wait_until(Duration::from_secs(30), || connection.is_link_proven()).await,
        "the coordinator should answer this node's heartbeats"
    );

    let shutdown = node.shutdown_token();
    coordinator.kill().await;

    assert!(
        timeout(Duration::from_secs(10), shutdown.cancelled())
            .await
            .is_ok(),
        "a Reliable node must still shut down when it loses the coordinator"
    );
}

/// A coordinator that welcomes a node and then dies before ever answering it.
///
/// This is the case that used to hang forever: the disconnect detector armed
/// only on the first heartbeat echo, so a coordinator that never echoed left
/// the node waiting indefinitely with no way to notice or recover.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn coordinator_that_never_answers_is_still_noticed() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let node = Node::create(
        NodeConfig::new(unique_name("unanswered"))
            .mode(Qos::Reliable)
            .coord_mode(CoordMode::External)
            .timing(test_timing()),
    )
    .await
    .expect("node should register with the running coordinator");

    // Killed immediately — deliberately without waiting for the link to be
    // proven, so the detector is still unarmed.
    let connection = node.connection();
    coordinator.kill().await;

    let shutdown = node.shutdown_token();
    assert!(
        timeout(Duration::from_secs(30), shutdown.cancelled())
            .await
            .is_ok(),
        "a node must notice a coordinator that never answered it, not wait forever"
    );
    assert!(
        !connection.is_link_proven(),
        "this test is only meaningful while the link was never proven"
    );
}
