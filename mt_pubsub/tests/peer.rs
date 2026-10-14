use mt_pubsub::{CoordMode, Node, NodeConfig, Qos};
use std::{net::TcpListener, time::Duration};

fn free_endpoint() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    format!("tcp/{}", listener.local_addr().unwrap())
}
async fn seed() -> (Node, String) {
    let endpoint = free_endpoint();
    let node = Node::create(
        NodeConfig::new("seed")
            .mode(Qos::BestEffort)
            .local_only(true)
            .listen_endpoint(endpoint.clone())
            .peer_endpoints(vec![]),
    )
    .await
    .unwrap();
    (node, endpoint)
}
async fn peer(name: &str, endpoint: &str) -> Node {
    Node::create(
        NodeConfig::new(name)
            .mode(Qos::BestEffort)
            .local_only(true)
            .peer_endpoints(vec![endpoint.to_owned()]),
    )
    .await
    .unwrap()
}
async fn receive<T: mt_sea::Sendable>(subscriber: &mut mt_pubsub::Subscriber<T>) -> T {
    tokio::time::timeout(Duration::from_secs(5), subscriber.next())
        .await
        .unwrap()
        .unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn peers_exchange_all_delivery_modes_without_a_coordinator() {
    assert!(matches!(
        NodeConfig::new("default").coordinator_mode(),
        CoordMode::Peer
    ));
    let (_seed, endpoint) = seed().await;
    let writer = peer("writer", &endpoint).await;
    let reader = peer("reader", &endpoint).await;
    for (index, qos) in [Qos::BestEffort, Qos::TryReliable, Qos::Reliable]
        .into_iter()
        .enumerate()
    {
        let topic = format!("/peer/mode/{index}");
        let mut sub = reader
            .create_subscriber::<String>(topic.clone(), 16, qos)
            .await
            .unwrap();
        let publisher = writer.create_publisher::<String>(topic, qos).await.unwrap();
        publisher
            .wait_for_subscribers(Duration::from_secs(5))
            .await
            .unwrap();
        for index in 0..10 {
            publisher
                .publish(&format!("message-{index}"))
                .await
                .unwrap();
        }
        for index in 0..10 {
            assert_eq!(receive(&mut sub).await, format!("message-{index}"));
        }
        sub.unsubscribe().await.unwrap();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn retained_samples_and_new_images_reach_late_subscribers() {
    let (_seed, endpoint) = seed().await;
    let writer = peer("image_writer", &endpoint).await;
    let reader = peer("image_reader", &endpoint).await;
    let publisher = writer
        .create_publisher::<mt_net::Image>("/peer/image".into(), Qos::Reliable.transient_local())
        .await
        .unwrap();
    let image = mt_net::Image {
        width: 2,
        height: 1,
        encoding: "rgb8".into(),
        step: 6,
        data: vec![1, 2, 3, 4, 5, 6],
        ..Default::default()
    };
    publisher.publish(&image).await.unwrap();
    let mut sub = reader
        .create_subscriber::<mt_net::Image>("/peer/image".into(), 8, Qos::Reliable)
        .await
        .unwrap();
    assert_eq!(receive(&mut sub).await, image);
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();
    publisher.publish(&image).await.unwrap();
    assert_eq!(receive(&mut sub).await, image);
    assert!(
        sub.try_next().is_none(),
        "retained/live overlap must not duplicate a sample"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reliable_acknowledges_enqueue_and_respects_backpressure() {
    let (_seed, endpoint) = seed().await;
    let writer = peer("blocked_writer", &endpoint).await;
    let reader = peer("blocked_reader", &endpoint).await;
    let mut sub = reader
        .create_subscriber::<String>("/peer/backpressure".into(), 1, Qos::Reliable)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/peer/backpressure".into(), Qos::Reliable)
        .await
        .unwrap();
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();
    publisher.publish(&"first".to_owned()).await.unwrap();
    let second = publisher.clone();
    let sending = tokio::spawn(async move { second.publish(&"second".to_owned()).await });
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(!sending.is_finished());
    assert_eq!(receive(&mut sub).await, "first");
    sending.await.unwrap().unwrap();
    assert_eq!(receive(&mut sub).await, "second");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn gossip_keeps_peers_connected_after_bootstrap_leaves() {
    let (bootstrap, endpoint) = seed().await;
    let writer = peer("mesh_writer", &endpoint).await;
    let reader = peer("mesh_reader", &endpoint).await;
    let mut sub = reader
        .create_subscriber::<String>("/peer/mesh".into(), 8, Qos::Reliable)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/peer/mesh".into(), Qos::Reliable)
        .await
        .unwrap();
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();
    publisher.publish(&"before".into()).await.unwrap();
    assert_eq!(receive(&mut sub).await, "before");
    drop(bootstrap);
    tokio::time::sleep(Duration::from_millis(200)).await;
    publisher.publish(&"after".into()).await.unwrap();
    assert_eq!(receive(&mut sub).await, "after");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn qos_and_types_are_checked_without_coordinator_registration() {
    let (_seed, endpoint) = seed().await;
    let writer = peer("type_writer", &endpoint).await;
    let reader = peer("type_reader", &endpoint).await;
    let _sub = reader
        .create_subscriber::<u64>("/peer/type".into(), 8, Qos::Reliable)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/peer/type".into(), Qos::Reliable)
        .await
        .unwrap();
    assert!(
        publisher
            .wait_for_subscribers(Duration::from_millis(150))
            .await
            .is_err()
    );
    let be = writer
        .create_publisher::<u64>("/peer/qos".into(), Qos::BestEffort)
        .await
        .unwrap();
    // Discovery is asynchronous: an already known incompatibility returns an
    // error immediately; a subsequently discovered one closes the subscription.
    if let Ok(mut incompatible) = reader
        .create_subscriber::<u64>("/peer/qos".into(), 8, Qos::Reliable)
        .await
    {
        assert!(
            tokio::time::timeout(Duration::from_secs(5), incompatible.next())
                .await
                .unwrap()
                .is_none()
        );
    }
    drop(be);
    assert!(
        reader
            .create_subscriber::<u64>("/peer/zero".into(), 0, Qos::BestEffort)
            .await
            .is_err()
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn losing_a_reliable_peer_is_still_fatal() {
    let (_seed, endpoint) = seed().await;
    let reader = peer("fatal_reader", &endpoint).await;
    let writer = Node::create(
        NodeConfig::new("fatal_writer")
            .local_only(true)
            .peer_endpoints(vec![endpoint]),
    )
    .await
    .unwrap();
    let mut sub = reader
        .create_subscriber::<String>("/peer/fatal".into(), 8, Qos::Reliable)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/peer/fatal".into(), Qos::Reliable)
        .await
        .unwrap();
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();
    publisher.publish(&"ready".into()).await.unwrap();
    assert_eq!(receive(&mut sub).await, "ready");
    let shutdown = reader.shutdown_token();
    drop(publisher);
    drop(writer);
    tokio::time::timeout(Duration::from_secs(5), shutdown.cancelled())
        .await
        .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn multicast_discovers_default_peers_without_a_tcp_coordinator() {
    let writer = Node::create(
        NodeConfig::new("scout_writer")
            .mode(Qos::BestEffort)
            .local_only(true),
    )
    .await
    .unwrap();
    let reader = Node::create(
        NodeConfig::new("scout_reader")
            .mode(Qos::BestEffort)
            .local_only(true),
    )
    .await
    .unwrap();
    let mut sub = reader
        .create_subscriber::<String>("/peer/scouting".into(), 8, Qos::Reliable)
        .await
        .unwrap();
    let publisher = writer
        .create_publisher::<String>("/peer/scouting".into(), Qos::Reliable)
        .await
        .unwrap();
    publisher
        .wait_for_subscribers(Duration::from_secs(5))
        .await
        .unwrap();
    publisher.publish(&"discovered".into()).await.unwrap();
    assert_eq!(receive(&mut sub).await, "discovered");
}
