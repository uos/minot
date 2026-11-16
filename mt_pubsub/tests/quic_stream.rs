//! Large best-effort samples over QUIC streams, with shared memory off.
use mt_pubsub::{Node, NodeConfig, PeerTransport, Qos};
use std::{net::UdpSocket, time::Duration};

fn free_endpoint() -> String {
    let listener = UdpSocket::bind("127.0.0.1:0").unwrap();
    format!("quic/{}", listener.local_addr().unwrap())
}
async fn node(name: &str, listen: Option<&str>, peers: Vec<String>) -> Node {
    let mut config = NodeConfig::new(name)
        .peer_transport(PeerTransport::Quic)
        .mode(Qos::BestEffort)
        .local_only(true)
        .peer_endpoints(peers);
    if let Some(endpoint) = listen {
        config = config.listen_endpoint(endpoint.to_owned());
    }
    Node::create(config).await.unwrap()
}

/// A sample published while the previous one is still going out is not lost:
/// the newest one waits and follows, so the last of a burst always arrives.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_burst_of_large_samples_ends_with_the_newest() {
    // SAFETY: set before any node exists; this test binary has no other test.
    unsafe { std::env::set_var("MINOT_SHM_DISABLED", "1") };
    let endpoint = free_endpoint();
    let _seed = node("seed", Some(&endpoint), vec![]).await;
    let writer = node("writer", None, vec![endpoint.clone()]).await;
    let reader = node("reader", None, vec![endpoint]).await;
    let large = |index: usize| format!("{index:04}:{}", "x".repeat(4 << 20));

    let mut sub = reader
        .create_subscriber::<String>("/stream/burst".to_owned(), 64, Qos::BestEffort)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/stream/burst".to_owned(), Qos::BestEffort)
        .await
        .unwrap();
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();

    let count = 20;
    for index in 0..count {
        publisher.publish(&large(index)).await.unwrap();
    }
    let mut received = Vec::new();
    while let Ok(Some(sample)) = tokio::time::timeout(Duration::from_secs(3), sub.next()).await {
        assert_eq!(sample.len(), large(0).len());
        received.push(sample[..4].parse::<usize>().unwrap());
        if received.last() == Some(&(count - 1)) {
            break;
        }
    }
    assert!(
        received.windows(2).all(|w| w[0] < w[1]),
        "out of order: {received:?}"
    );
    assert_eq!(
        received.last(),
        Some(&(count - 1)),
        "newest lost: {received:?}"
    );
}
