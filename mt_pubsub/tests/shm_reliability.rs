#![cfg(feature = "shm")]
/// Test suite managed by AI.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use mt_pubsub::{CoordMode, Node, NodeConfig, Qos};
use rkyv::{Archive, Deserialize, Serialize};
use tokio::time::timeout;

const OPERATION_TIMEOUT: Duration = Duration::from_secs(30);
const DEFAULT_SHM_POOL_SIZE: usize = 7 * 1024 * 1024;
const DEFAULT_SHM_THRESHOLD: usize = 1024 * 1024;
const WORKER_ROLE: &str = "MINOT_SHM_TEST_WORKER_ROLE";
const WORKER_DIR: &str = "MINOT_SHM_TEST_WORKER_DIR";
const WORKER_PREFIX: &str = "MINOT_SHM_TEST_WORKER_PREFIX";
const WORKER_PAYLOAD_SIZE: &str = "MINOT_SHM_TEST_PAYLOAD_SIZE";
const WORKER_MESSAGES: &str = "MINOT_SHM_TEST_MESSAGES";

static TEST_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

#[derive(Archive, Serialize, Deserialize, Debug)]
struct LargePacket {
    sequence: u64,
    payload: Vec<u8>,
}

struct Harness {
    publisher_node: Node,
    subscriber_node: Node,
    prefix: String,
}

impl Harness {
    async fn new() -> Self {
        Self::new_with_max_message_size(None).await
    }

    async fn new_with_max_message_size(max_message_size: Option<usize>) -> Self {
        // A coordinator from the previous test owns a background runtime. Give its router and
        // lock a brief chance to close before probing the fixed local coordinator endpoint.
        tokio::time::sleep(Duration::from_millis(500)).await;
        let _ = env_logger::builder().is_test(true).try_init();
        let unique = format!(
            "{}_{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let domain_id = (SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
            % (u16::MAX as u32 - 1)
            + 1) as u16;

        // SAFETY: Each integration-test invocation runs this file as its own process and this
        // setup happens before the test creates Minot/Zenoh threads.
        unsafe {
            std::env::set_var("MINOT_DOMAIN_ID", domain_id.to_string());
            std::env::remove_var("MINOT_SHM_DISABLED");
            std::env::set_var("MINOT_SHM_SIZE", DEFAULT_SHM_POOL_SIZE.to_string());
            std::env::set_var("MINOT_SHM_THRESHOLD", DEFAULT_SHM_THRESHOLD.to_string());
            match max_message_size {
                Some(size) => std::env::set_var("MINOT_SHM_MAX_MESSAGE_SIZE", size.to_string()),
                None => std::env::remove_var("MINOT_SHM_MAX_MESSAGE_SIZE"),
            }
            std::env::set_var("MINOT_SHM_ALLOCATION_TIMEOUT_MS", "250");
        }

        let subscriber_node = timeout(
            OPERATION_TIMEOUT,
            Node::create(
                NodeConfig::new(format!("shm_sub_{unique}"))
                    .coord_mode(CoordMode::Start)
                    .local_only(true),
            ),
        )
        .await
        .expect("subscriber node creation timed out")
        .expect("subscriber node creation failed");

        let publisher_node = timeout(
            OPERATION_TIMEOUT,
            Node::create(
                NodeConfig::new(format!("shm_pub_{unique}"))
                    .coord_mode(CoordMode::External)
                    .local_only(true),
            ),
        )
        .await
        .expect("publisher node creation timed out")
        .expect("publisher node creation failed");

        Self {
            publisher_node,
            subscriber_node,
            prefix: unique,
        }
    }

    async fn run_stream(&self, label: &str, payload_size: usize, messages: usize) -> Duration {
        let topic = format!("/shm_test/{}/{label}", self.prefix);
        let mut subscriber = timeout(
            OPERATION_TIMEOUT,
            self.subscriber_node.create_subscriber::<LargePacket>(
                topic.clone(),
                messages.max(1),
                Qos::Reliable,
            ),
        )
        .await
        .expect("subscriber creation timed out")
        .expect("subscriber creation failed");
        let publisher = timeout(
            OPERATION_TIMEOUT,
            self.publisher_node
                .create_publisher::<LargePacket>(topic, Qos::Reliable),
        )
        .await
        .expect("publisher creation timed out")
        .expect("publisher creation failed");

        let started = Instant::now();
        for sequence in 0..messages as u64 {
            let fill = (sequence % 251) as u8;
            let packet = LargePacket {
                sequence,
                payload: vec![fill; payload_size],
            };

            timeout(OPERATION_TIMEOUT, publisher.publish(&packet))
                .await
                .unwrap_or_else(|_| {
                    panic!("publish timed out for sequence {sequence}, size {payload_size}")
                })
                .unwrap_or_else(|error| {
                    panic!("publish failed for sequence {sequence}, size {payload_size}: {error}")
                });

            let received = timeout(OPERATION_TIMEOUT, subscriber.next())
                .await
                .unwrap_or_else(|_| {
                    panic!("receive timed out for sequence {sequence}, size {payload_size}")
                })
                .unwrap_or_else(|| {
                    panic!("subscriber closed for sequence {sequence}, size {payload_size}")
                });
            assert_eq!(received.sequence, sequence);
            assert_eq!(received.payload.len(), payload_size);
            assert!(received.payload.iter().all(|byte| *byte == fill));
        }
        started.elapsed()
    }

    async fn run_archived_message(&self, payload_size: usize) {
        let topic = format!("/shm_test/{}/archived", self.prefix);
        let mut subscriber = self
            .subscriber_node
            .create_subscriber::<LargePacket>(topic.clone(), 1, Qos::Reliable)
            .await
            .expect("archived subscriber creation failed");
        let publisher = self
            .publisher_node
            .create_publisher::<LargePacket>(topic, Qos::Reliable)
            .await
            .expect("archived publisher creation failed");
        let packet = LargePacket {
            sequence: 42,
            payload: vec![7; payload_size],
        };

        publisher.publish(&packet).await.expect("publish failed");
        let message = timeout(OPERATION_TIMEOUT, subscriber.next_archived())
            .await
            .expect("archived receive timed out")
            .expect("archived subscriber closed");
        let archived = message.archived();
        assert_eq!(archived.sequence, 42);
        assert_eq!(archived.payload.len(), payload_size);
        assert!(archived.payload.iter().all(|byte| *byte == 7));

        let owned = message.deserialize().expect("explicit deserialize failed");
        assert_eq!(owned.sequence, 42);
        assert_eq!(owned.payload.len(), payload_size);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reliable_large_packets_use_shm_without_fallback() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new().await;
    mt_sea::client::reset_shm_transfer_stats();

    let cases = [
        ("threshold", 1024 * 1024, 25),
        ("medium", 2 * 1024 * 1024, 10),
        ("large", 8 * 1024 * 1024, 2),
    ];
    let expected_messages: u64 = cases.iter().map(|(_, _, count)| *count as u64).sum();

    for (label, payload_size, messages) in cases {
        harness.run_stream(label, payload_size, messages).await;
    }

    let stats = mt_sea::client::shm_transfer_stats();
    assert_eq!(stats.send_attempts, expected_messages, "{stats:?}");
    assert_eq!(stats.send_successes, expected_messages, "{stats:?}");
    assert_eq!(stats.receives, expected_messages, "{stats:?}");
    assert_eq!(stats.send_fallbacks, 0, "SHM silently fell back: {stats:?}");
    assert!(stats.send_bytes > 0, "{stats:?}");
    assert_eq!(stats.send_bytes, stats.receive_bytes, "{stats:?}");
    assert_eq!(stats.network_send_bytes, 0, "{stats:?}");
    assert_eq!(stats.network_receive_bytes, 0, "{stats:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn archived_subscriber_borrows_validated_payload() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new().await;
    harness.run_archived_message(2 * 1024 * 1024).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn payload_below_threshold_uses_non_shm_transport() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new().await;
    mt_sea::client::reset_shm_transfer_stats();

    harness
        .run_stream("below_shm_threshold", 256 * 1024, 1)
        .await;

    let stats = mt_sea::client::shm_transfer_stats();
    assert_eq!(stats.send_attempts, 0, "{stats:?}");
    assert_eq!(stats.send_successes, 0, "{stats:?}");
    assert_eq!(stats.network_sends, 1, "{stats:?}");
    assert_eq!(stats.network_receives, 1, "{stats:?}");
    assert_eq!(
        stats.network_send_bytes, stats.network_receive_bytes,
        "{stats:?}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reliable_large_packets_use_shm_across_processes() {
    let _guard = TEST_LOCK.lock().await;
    run_cross_process(false).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn shm_sender_falls_back_to_network_for_shm_disabled_process() {
    let _guard = TEST_LOCK.lock().await;
    run_cross_process(true).await;
}

async fn run_cross_process(disable_subscriber_shm: bool) {
    tokio::time::sleep(Duration::from_millis(500)).await;
    let unique = format!(
        "{}_{}_{}",
        std::process::id(),
        if disable_subscriber_shm {
            "mixed"
        } else {
            "shm"
        },
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    );
    let domain_id = (SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .subsec_nanos()
        % (u16::MAX as u32 - 1)
        + 1) as u16;
    let worker_dir = std::env::temp_dir().join(format!("minot_shm_test_{unique}"));
    std::fs::create_dir(&worker_dir).expect("failed to create worker directory");

    let executable = std::env::current_exe().expect("failed to locate integration test binary");
    let configure = |command: &mut tokio::process::Command, role: &str| {
        command
            .arg("--exact")
            .arg("shm_test_worker")
            .arg("--nocapture")
            .env(WORKER_ROLE, role)
            .env(WORKER_DIR, &worker_dir)
            .env(WORKER_PREFIX, &unique)
            .env(WORKER_PAYLOAD_SIZE, (1024 * 1024).to_string())
            .env(WORKER_MESSAGES, "25")
            .env("MINOT_DOMAIN_ID", domain_id.to_string())
            .env("MINOT_SHM_SIZE", DEFAULT_SHM_POOL_SIZE.to_string())
            .env("MINOT_SHM_THRESHOLD", DEFAULT_SHM_THRESHOLD.to_string())
            .env_remove("MINOT_SHM_MAX_MESSAGE_SIZE")
            .env("MINOT_SHM_ALLOCATION_TIMEOUT_MS", "250");
        if disable_subscriber_shm && role == "subscriber" {
            command.env("MINOT_SHM_DISABLED", "1");
        } else {
            command.env_remove("MINOT_SHM_DISABLED");
        }
    };

    let mut subscriber_command = tokio::process::Command::new(&executable);
    configure(&mut subscriber_command, "subscriber");
    let mut subscriber_child = subscriber_command
        .spawn()
        .expect("failed to spawn subscriber worker");

    let ready_path = worker_dir.join("subscriber.ready");
    timeout(OPERATION_TIMEOUT, async {
        while !ready_path.exists() {
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .expect("subscriber worker did not become ready");

    let mut publisher_command = tokio::process::Command::new(&executable);
    configure(&mut publisher_command, "publisher");
    let publisher_status = timeout(OPERATION_TIMEOUT, publisher_command.status())
        .await
        .expect("publisher worker timed out")
        .expect("failed to wait for publisher worker");
    assert!(publisher_status.success(), "publisher worker failed");

    let subscriber_status = timeout(OPERATION_TIMEOUT, subscriber_child.wait())
        .await
        .expect("subscriber worker timed out")
        .expect("failed to wait for subscriber worker");
    assert!(subscriber_status.success(), "subscriber worker failed");

    let publisher_stats = read_stats(&worker_dir.join("publisher.stats"));
    let subscriber_stats = read_stats(&worker_dir.join("subscriber.stats"));
    assert_eq!(publisher_stats.send_attempts, 25, "{publisher_stats:?}");
    assert_eq!(publisher_stats.send_successes, 25, "{publisher_stats:?}");
    assert_eq!(publisher_stats.send_fallbacks, 0, "{publisher_stats:?}");
    assert!(publisher_stats.send_bytes > 0, "{publisher_stats:?}");
    if disable_subscriber_shm {
        assert_eq!(subscriber_stats.receives, 0, "{subscriber_stats:?}");
        assert_eq!(
            subscriber_stats.network_receives, 25,
            "{subscriber_stats:?}"
        );
        assert!(
            subscriber_stats.network_receive_bytes > 0,
            "{subscriber_stats:?}"
        );
    } else {
        assert_eq!(subscriber_stats.receives, 25, "{subscriber_stats:?}");
        assert_eq!(
            publisher_stats.send_bytes, subscriber_stats.receive_bytes,
            "publisher={publisher_stats:?}, subscriber={subscriber_stats:?}"
        );
    }

    std::fs::remove_dir_all(&worker_dir).expect("failed to remove worker directory");
}

/// Entry point used by `reliable_large_packets_use_shm_across_processes`.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn shm_test_worker() {
    let Ok(role) = std::env::var(WORKER_ROLE) else {
        return;
    };
    let worker_dir = std::path::PathBuf::from(std::env::var_os(WORKER_DIR).unwrap());
    let prefix = std::env::var(WORKER_PREFIX).unwrap();
    let payload_size = std::env::var(WORKER_PAYLOAD_SIZE)
        .unwrap()
        .parse::<usize>()
        .unwrap();
    let messages = std::env::var(WORKER_MESSAGES)
        .unwrap()
        .parse::<usize>()
        .unwrap();
    let topic = format!("/shm_process_test/{prefix}");
    mt_sea::network::set_local_only(true);
    mt_sea::client::reset_shm_transfer_stats();

    match role.as_str() {
        "subscriber" => {
            let node = timeout(
                OPERATION_TIMEOUT,
                Node::create(
                    NodeConfig::new(format!("shm_process_sub_{prefix}"))
                        .coord_mode(CoordMode::Start)
                        .local_only(true),
                ),
            )
            .await
            .expect("subscriber worker node creation timed out")
            .expect("subscriber worker node creation failed");
            let mut subscriber = timeout(
                OPERATION_TIMEOUT,
                node.create_subscriber::<LargePacket>(topic, messages, Qos::Reliable),
            )
            .await
            .expect("subscriber worker registration timed out")
            .expect("subscriber worker registration failed");
            std::fs::write(worker_dir.join("subscriber.ready"), b"ready")
                .expect("failed to signal subscriber readiness");

            for sequence in 0..messages as u64 {
                let packet = timeout(OPERATION_TIMEOUT, subscriber.next())
                    .await
                    .expect("subscriber worker receive timed out")
                    .expect("subscriber worker closed");
                let fill = (sequence % 251) as u8;
                assert_eq!(packet.sequence, sequence);
                assert_eq!(packet.payload.len(), payload_size);
                assert!(packet.payload.iter().all(|byte| *byte == fill));
            }
            write_stats(&worker_dir.join("subscriber.stats"));
        }
        "publisher" => {
            let node = timeout(
                OPERATION_TIMEOUT,
                Node::create(
                    NodeConfig::new(format!("shm_process_pub_{prefix}"))
                        .coord_mode(CoordMode::External)
                        .local_only(true),
                ),
            )
            .await
            .expect("publisher worker node creation timed out")
            .expect("publisher worker node creation failed");
            let publisher = timeout(
                OPERATION_TIMEOUT,
                node.create_publisher::<LargePacket>(topic, Qos::Reliable),
            )
            .await
            .expect("publisher worker registration timed out")
            .expect("publisher worker registration failed");

            for sequence in 0..messages as u64 {
                let fill = (sequence % 251) as u8;
                let packet = LargePacket {
                    sequence,
                    payload: vec![fill; payload_size],
                };
                timeout(OPERATION_TIMEOUT, publisher.publish(&packet))
                    .await
                    .expect("publisher worker publish timed out")
                    .expect("publisher worker publish failed");
            }
            write_stats(&worker_dir.join("publisher.stats"));
        }
        other => panic!("unknown SHM worker role: {other}"),
    }
}

fn write_stats(path: &std::path::Path) {
    let stats = mt_sea::client::shm_transfer_stats();
    std::fs::write(
        path,
        format!(
            "{},{},{},{},{},{},{},{},{},{}",
            stats.send_attempts,
            stats.send_successes,
            stats.send_fallbacks,
            stats.receives,
            stats.send_bytes,
            stats.receive_bytes,
            stats.network_sends,
            stats.network_send_bytes,
            stats.network_receives,
            stats.network_receive_bytes,
        ),
    )
    .expect("failed to write SHM stats");
}

fn read_stats(path: &std::path::Path) -> mt_sea::client::ShmTransferStats {
    let raw = std::fs::read_to_string(path).expect("failed to read SHM stats");
    let mut fields = raw.split(',').map(|field| field.parse::<u64>().unwrap());
    mt_sea::client::ShmTransferStats {
        send_attempts: fields.next().unwrap(),
        send_successes: fields.next().unwrap(),
        send_fallbacks: fields.next().unwrap(),
        receives: fields.next().unwrap(),
        send_bytes: fields.next().unwrap(),
        receive_bytes: fields.next().unwrap(),
        network_sends: fields.next().unwrap(),
        network_send_bytes: fields.next().unwrap(),
        network_receives: fields.next().unwrap(),
        network_receive_bytes: fields.next().unwrap(),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn payload_larger_than_pool_grows_provider() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new().await;
    mt_sea::client::reset_shm_transfer_stats();

    harness.run_stream("pool_growth", 8 * 1024 * 1024, 2).await;

    let stats = mt_sea::client::shm_transfer_stats();
    assert_eq!(stats.send_attempts, 2, "{stats:?}");
    assert_eq!(stats.send_successes, 2, "{stats:?}");
    assert_eq!(stats.receives, 2, "{stats:?}");
    assert_eq!(stats.send_fallbacks, 0, "SHM silently fell back: {stats:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn payload_above_shm_limit_falls_back_without_hanging() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new_with_max_message_size(Some(1024 * 1024)).await;
    mt_sea::client::reset_shm_transfer_stats();

    harness
        .run_stream("too_large_for_shm", 2 * 1024 * 1024, 1)
        .await;

    let stats = mt_sea::client::shm_transfer_stats();
    assert_eq!(stats.send_attempts, 1, "{stats:?}");
    assert_eq!(stats.send_successes, 0, "{stats:?}");
    assert_eq!(stats.receives, 0, "{stats:?}");
    assert_eq!(stats.send_fallbacks, 1, "{stats:?}");
    assert_eq!(stats.send_bytes, 0, "{stats:?}");
    assert_eq!(stats.network_sends, 1, "{stats:?}");
    assert!(stats.network_send_bytes > 2 * 1024 * 1024, "{stats:?}");
    assert_eq!(stats.network_receives, 1, "{stats:?}");
    assert_eq!(
        stats.network_send_bytes, stats.network_receive_bytes,
        "{stats:?}"
    );
}

/// A repeatable end-to-end benchmark. Run with:
/// `cargo test -p mt_pubsub --features shm --test shm_reliability --release \
/// shm_large_packet_benchmark -- --exact --ignored --nocapture`
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
#[ignore = "manual SHM throughput benchmark"]
async fn shm_large_packet_benchmark() {
    let _guard = TEST_LOCK.lock().await;
    let harness = Harness::new().await;
    mt_sea::client::reset_shm_transfer_stats();

    let cases = [
        ("1m", 1024 * 1024, 100),
        ("4m", 4 * 1024 * 1024, 20),
        ("16m", 16 * 1024 * 1024, 5),
    ];

    eprintln!("payload\tmessages\ttotal_ms\tmsg_per_s\tMiB_per_s");
    for (label, payload_size, messages) in cases {
        let elapsed = harness.run_stream(label, payload_size, messages).await;
        let seconds = elapsed.as_secs_f64();
        let message_rate = messages as f64 / seconds;
        let mib_rate = payload_size as f64 * messages as f64 / (1024.0 * 1024.0) / seconds;
        eprintln!(
            "{}\t{}\t{:.3}\t{:.1}\t{:.1}",
            payload_size,
            messages,
            elapsed.as_secs_f64() * 1000.0,
            message_rate,
            mib_rate
        );
    }

    let stats = mt_sea::client::shm_transfer_stats();
    eprintln!("SHM stats: {stats:?}");
    assert_eq!(stats.send_fallbacks, 0, "SHM silently fell back: {stats:?}");
    assert_eq!(stats.send_attempts, stats.send_successes, "{stats:?}");
    assert_eq!(stats.send_attempts, stats.receives, "{stats:?}");
}
