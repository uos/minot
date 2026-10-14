//! Separate processes and separate temporary filesystems model container starts.
use std::{
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn peer_worker() {
    let Ok(directory) = std::env::var("MINOT_TEST_PEER_DIR") else {
        return;
    };
    let directory = std::path::PathBuf::from(directory);
    let index: usize = std::env::var("MINOT_TEST_PEER_INDEX")
        .unwrap()
        .parse()
        .unwrap();
    let endpoint = std::env::var("MINOT_TEST_PEER_ENDPOINT").unwrap();
    let swarm = std::env::var("MINOT_TEST_PEER_SWARM").is_ok();
    let count = if swarm { 32 } else { 8 };
    let qos = if swarm {
        mt_pubsub::Qos::BestEffort
    } else {
        mt_pubsub::Qos::Reliable
    };
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        let deadline = Instant::now() + Duration::from_secs(60);
        while !directory.join("start").exists() {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let config = mt_pubsub::NodeConfig::new(format!("peer_{index}"))
            .mode(mt_pubsub::Qos::BestEffort)
            .local_only(true);
        let config = if swarm {
            config
        } else if index == 0 {
            config.listen_endpoint(endpoint).peer_endpoints(vec![])
        } else {
            config.peer_endpoints(vec![endpoint])
        };
        let node = mt_pubsub::Node::create(config).await.unwrap();
        // Match the swarm application's separate map and TF nodes: 64 peer
        // sessions discover one another during a simultaneous process launch.
        let _tf_node = if swarm {
            Some(
                mt_pubsub::Node::create(
                    mt_pubsub::NodeConfig::new(format!("peer_{index}_tf"))
                        .mode(mt_pubsub::Qos::BestEffort)
                        .local_only(true),
                )
                .await
                .unwrap(),
            )
        } else {
            None
        };
        let mut subscriber = node
            .create_subscriber::<String>("/startup".into(), count * 2, qos)
            .await
            .unwrap();
        let publisher = node
            .create_publisher::<String>("/startup".into(), qos)
            .await
            .unwrap();
        publisher
            .wait_for_subscriber_count(count, Duration::from_secs(30))
            .await
            .unwrap();
        std::fs::write(directory.join(format!("ready_{index}")), b"ready").unwrap();
        while !directory.join("publish").exists() {
            assert!(Instant::now() < deadline);
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        publisher.publish(&format!("peer_{index}")).await.unwrap();
        let mut received = std::collections::HashSet::new();
        for _ in 0..count {
            let value = tokio::time::timeout(Duration::from_secs(8), subscriber.next())
                .await
                .unwrap()
                .unwrap();
            received.insert(value);
        }
        assert_eq!(
            received,
            (0..count)
                .map(|i| format!("peer_{i}"))
                .collect::<std::collections::HashSet<_>>()
        );
        assert!(
            std::fs::read_dir(std::env::temp_dir())
                .unwrap()
                .all(|entry| !entry
                    .unwrap()
                    .file_name()
                    .to_string_lossy()
                    .starts_with("minot-coord")),
            "peer startup must never acquire a coordinator lock"
        );
        drop(publisher);
        drop(subscriber);
        node.shutdown_token().cancel();
        if let Some(tf_node) = &_tf_node {
            tf_node.shutdown_token().cancel();
        }
        // Let the asynchronous session-close tasks finish before the worker's
        // Tokio runtime is destroyed.
        tokio::time::sleep(Duration::from_millis(200)).await;
    });
}
struct Workers(Vec<Child>);
impl Drop for Workers {
    fn drop(&mut self) {
        for child in &mut self.0 {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[test]
fn concurrent_peers_with_separate_filesystems_exchange_messages() {
    concurrent_startup(false);
}

#[test]
fn simultaneous_swarm_startup_with_two_nodes_per_process() {
    concurrent_startup(true);
}

fn concurrent_startup(swarm: bool) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = listener.local_addr().unwrap();
    let count = if swarm { 32 } else { 8 };
    let directory =
        std::env::temp_dir().join(format!("minot-peer-startup-{}-{count}", std::process::id()));
    std::fs::create_dir(&directory).unwrap();
    let mut workers = Workers(vec![]);
    for index in 0..count {
        let temporary = directory.join(format!("worker_{index}"));
        std::fs::create_dir(&temporary).unwrap();
        let mut command = Command::new(std::env::current_exe().unwrap());
        command
            .args(["--exact", "peer_worker", "--nocapture"])
            .env("MINOT_TEST_PEER_DIR", &directory)
            .env("MINOT_TEST_PEER_INDEX", index.to_string())
            .env("MINOT_TEST_PEER_ENDPOINT", format!("tcp/{address}"))
            .env("MINOT_ZENOH_PORT", address.port().to_string())
            .env("MINOT_DOMAIN_ID", address.port().to_string())
            .env("TMPDIR", temporary)
            .env("ZENOH_RUNTIME", "(net: (worker_threads: 1))")
            .stdout(Stdio::null())
            .stderr(Stdio::inherit());
        if swarm {
            command.env("MINOT_TEST_PEER_SWARM", "1");
        }
        workers.0.push(command.spawn().unwrap());
    }
    drop(listener);
    std::fs::write(directory.join("start"), b"start").unwrap();
    let deadline = Instant::now() + Duration::from_secs(45);
    loop {
        for child in &mut workers.0 {
            if let Some(status) = child.try_wait().unwrap() {
                panic!("peer worker exited early: {status}");
            }
        }
        if (0..count).all(|i| directory.join(format!("ready_{i}")).exists()) {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "peers did not start without a coordinator"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    std::fs::write(directory.join("publish"), b"publish").unwrap();
    for child in &mut workers.0 {
        assert!(child.wait().unwrap().success());
    }
    std::fs::remove_dir_all(directory).unwrap();
}
