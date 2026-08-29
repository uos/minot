//! A node must survive losing its coordinator.
//!
//! These tests kill a real coordinator process mid-stream and assert that the
//! publisher and subscriber the caller is holding keep working once it comes
//! back.
//!
//! The coordinator runs as a subprocess so it can be `SIGKILL`ed. An embedded
//! one shares this process and cannot be killed the way a real network
//! partition kills it.

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
    /// Poll readiness because coordinator startup time varies with load.
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
        // The blocking kill completes cleanup here, so the process cannot leak
        // out of the test run.
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Locate the `minot` binary built alongside this test.
///
/// Returns `None` when it was not built. The coordinator lives in the `minot`
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

/// Publish until one lands or the budget runs out. Best-effort may drop any
/// individual sample, so a single publish proves nothing either way.
async fn deliver_one(
    publisher: &mt_pubsub::Publisher<Sample>,
    subscriber: &mut mt_pubsub::Subscriber<Sample>,
    sequence: u64,
    budget: Duration,
) -> bool {
    let deadline = tokio::time::Instant::now() + budget;
    while tokio::time::Instant::now() < deadline {
        let _ = publisher.publish(&Sample { sequence }).await;
        if timeout(Duration::from_millis(200), subscriber.next())
            .await
            .ok()
            .flatten()
            .is_some()
        {
            return true;
        }
    }
    false
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
    // node's disconnect detector arms, so killing the coordinator before then
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
/// The disconnect detector arms on the first heartbeat echo, so a coordinator
/// that never echoes must still be noticed through the startup grace period.
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

    // Killed immediately, without waiting for the link to be
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

/// Reconnecting is not the point; receiving again is.
///
/// Every other test here asserts connection *state* — generation, connected,
/// link proven — and a node can satisfy all three while no data reaches its
/// subscriber, since the routes publishers hold are built from the connected
/// clients and a node that dropped is no longer among them. A viewer sees that
/// gap as "reconnected, but the view never updates again".
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_subscriber_receives_again_after_a_coordinator_restart() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let topic = unique_name("delivery");

    // Best-effort on both sides: the mode a viewer uses, and the one the
    // coordinator requires of a subscriber to a best-effort publisher.
    let make = |name: String| {
        Node::create(
            NodeConfig::new(name)
                .mode(Qos::BestEffort)
                .coord_mode(CoordMode::External)
                .timing(test_timing()),
        )
    };

    let publisher_node = make(unique_name("delivery_tx"))
        .await
        .expect("publisher should register");
    let subscriber_node = make(unique_name("delivery_rx"))
        .await
        .expect("subscriber should register");

    let publisher = publisher_node
        .create_publisher::<Sample>(topic.clone(), Qos::BestEffort)
        .await
        .expect("publisher should be created");
    let mut subscriber = subscriber_node
        .create_subscriber::<Sample>(topic.clone(), 10, Qos::BestEffort)
        .await
        .expect("subscriber should be created");

    assert!(
        deliver_one(&publisher, &mut subscriber, 1, Duration::from_secs(30)).await,
        "the subscriber should receive before anything is disturbed"
    );

    let connection = subscriber_node.connection();
    assert!(
        wait_until(Duration::from_secs(30), || connection.is_link_proven()).await,
        "the subscriber's link should be proven before the coordinator is killed"
    );

    coordinator.kill().await;
    assert!(
        wait_until(Duration::from_secs(10), || !connection.is_connected()).await,
        "the subscriber should notice the coordinator is gone"
    );

    let mut replacement = Coordinator::start()
        .await
        .expect("coordinator should restart");

    assert!(
        wait_until(Duration::from_secs(30), || connection.is_connected()).await,
        "the subscriber should re-register once the coordinator returns"
    );

    // The assertion this test exists for; everything above is setup.
    assert!(
        deliver_one(&publisher, &mut subscriber, 2, Duration::from_secs(60)).await,
        "the subscriber must receive again after the reconnect — reconnecting \
         without delivery resuming is the failure a viewer actually sees"
    );

    replacement.kill().await;
}

/// The viewer's shape: only the *subscriber* goes away. The coordinator keeps
/// running with its stale idea of that client, and the publisher's link is
/// never disturbed, so it never re-registers or re-derives its routes.
/// Everything rests on the coordinator re-pushing routes when the subscriber
/// returns, which it does only when it recognises the client as one it already
/// has rules for.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_returning_subscriber_is_routed_to_again_while_the_publisher_stays_up() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let topic = unique_name("returning");
    // Stable across the disappearance: the coordinator keys its rules by name,
    // so a viewer that comes back is the *same* client as far as it knows.
    let subscriber_name = unique_name("returning_rx");

    let make = |name: String| {
        Node::create(
            NodeConfig::new(name)
                .mode(Qos::BestEffort)
                .coord_mode(CoordMode::External)
                .timing(test_timing()),
        )
    };

    let publisher_node = make(unique_name("returning_tx"))
        .await
        .expect("publisher should register");
    let publisher = publisher_node
        .create_publisher::<Sample>(topic.clone(), Qos::BestEffort)
        .await
        .expect("publisher should be created");

    {
        let subscriber_node = make(subscriber_name.clone())
            .await
            .expect("subscriber should register");
        let mut subscriber = subscriber_node
            .create_subscriber::<Sample>(topic.clone(), 10, Qos::BestEffort)
            .await
            .expect("subscriber should be created");

        assert!(
            deliver_one(&publisher, &mut subscriber, 1, Duration::from_secs(30)).await,
            "the subscriber should receive before it goes away"
        );
    }
    // Gone. The coordinator still holds its rules, because a best-effort node
    // keeps them on purpose.

    // Long enough for the coordinator to have noticed and rebuilt the
    // publisher's routes without this subscriber in them.
    tokio::time::sleep(Duration::from_secs(3)).await;

    let subscriber_node = make(subscriber_name)
        .await
        .expect("subscriber should register again under the same name");
    let mut subscriber = subscriber_node
        .create_subscriber::<Sample>(topic.clone(), 10, Qos::BestEffort)
        .await
        .expect("subscriber should be created again");

    assert!(
        deliver_one(&publisher, &mut subscriber, 2, Duration::from_secs(60)).await,
        "a subscriber that comes back must be routed to again — the publisher was \
         never disturbed, so only the coordinator can put it back in the targets"
    );

    coordinator.kill().await;
}

/// The case the coordinator's re-registration branch exists for: a publisher
/// that was away while the topic gained a subscriber.
///
/// A non-Reliable publisher keeps its rules across a disconnect
/// (`removes_rules_on_exit`) and its cached route with them, so it comes back
/// publishing to the targets it left with. The route push that would have told
/// it about a second subscriber was skipped while it was away, so only the
/// coordinator can put that right when it re-registers.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_returning_publisher_learns_about_a_subscriber_that_joined_while_it_was_away() {
    let _guard = COORDINATOR_PORT.lock().await;
    let _ = env_logger::builder().is_test(true).try_init();
    mt_sea::network::set_local_only(true);

    let Some(mut coordinator) = Coordinator::start().await else {
        eprintln!("skipping: the `minot` binary was not built or did not start");
        return;
    };

    let topic = unique_name("late_sub");
    // Stable across the disappearance: the coordinator keys its rules by name.
    let publisher_name = unique_name("late_sub_tx");

    let make = |name: String| {
        Node::create(
            NodeConfig::new(name)
                .mode(Qos::BestEffort)
                .coord_mode(CoordMode::External)
                .timing(test_timing()),
        )
    };

    let early_node = make(unique_name("late_sub_rx_early"))
        .await
        .expect("the first subscriber should register");
    let mut early = early_node
        .create_subscriber::<Sample>(topic.clone(), 10, Qos::BestEffort)
        .await
        .expect("the first subscriber should be created");

    {
        let publisher_node = make(publisher_name.clone())
            .await
            .expect("publisher should register");
        let publisher = publisher_node
            .create_publisher::<Sample>(topic.clone(), Qos::BestEffort)
            .await
            .expect("publisher should be created");

        assert!(
            deliver_one(&publisher, &mut early, 1, Duration::from_secs(30)).await,
            "the publisher should reach the first subscriber before it goes away"
        );
    }
    // The publisher is gone. Its rules and its cached route survive.

    // Long enough for the coordinator to have noticed the publisher left.
    tokio::time::sleep(Duration::from_secs(3)).await;

    // The topic gains a second subscriber while the publisher cannot be told.
    let late_node = make(unique_name("late_sub_rx_late"))
        .await
        .expect("the second subscriber should register");
    let mut late = late_node
        .create_subscriber::<Sample>(topic.clone(), 10, Qos::BestEffort)
        .await
        .expect("the second subscriber should be created");

    let publisher_node = make(publisher_name)
        .await
        .expect("publisher should register again under the same name");
    let publisher = publisher_node
        .create_publisher::<Sample>(topic.clone(), Qos::BestEffort)
        .await
        .expect("publisher should be created again");

    assert!(
        deliver_one(&publisher, &mut late, 2, Duration::from_secs(60)).await,
        "the returning publisher must be told about the subscriber that joined while \
         it was away — it comes back with the target list it left with"
    );

    coordinator.kill().await;
}
