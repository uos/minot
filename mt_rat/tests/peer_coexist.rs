use std::{process::Command, time::Duration};

#[test]
fn coordinated_rat_worker() {
    if std::env::var_os("MINOT_TEST_RAT_COEXIST").is_none() {
        return;
    }
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        mt_sea::network::set_local_only(true);
        assert!(
            mt_coord::ensure_default_coordinator_ready(None)
                .await
                .unwrap()
        );
        rat::init_async("coexisting_rat", Some(Duration::from_secs(5)))
            .await
            .unwrap();
        let mut value = 42_u64;
        tokio::time::timeout(
            Duration::from_secs(5),
            rat::bacon_async(
                "unchanged_variable",
                &mut value,
                rat::VariableType::StaticOnly,
            ),
        )
        .await
        .unwrap()
        .unwrap();
        assert_eq!(value, 42);
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/{}", listener.local_addr().unwrap());
        drop(listener);
        let writer = mt_pubsub::Node::create(
            mt_pubsub::NodeConfig::new("coexisting_pubsub_writer")
                .mode(mt_pubsub::Qos::BestEffort)
                .listen_endpoint(endpoint.clone())
                .peer_endpoints(vec![]),
        )
        .await
        .unwrap();
        let reader = mt_pubsub::Node::create(
            mt_pubsub::NodeConfig::new("coexisting_pubsub_reader")
                .mode(mt_pubsub::Qos::BestEffort)
                .peer_endpoints(vec![endpoint]),
        )
        .await
        .unwrap();
        let mut sub = reader
            .create_subscriber::<String>("/coexist".into(), 8, mt_pubsub::Qos::Reliable)
            .await
            .unwrap();
        let publisher = writer
            .create_publisher::<String>("/coexist".into(), mt_pubsub::Qos::Reliable)
            .await
            .unwrap();
        publisher
            .wait_for_subscribers(Duration::from_secs(5))
            .await
            .unwrap();
        publisher.publish(&"peer data".into()).await.unwrap();
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(5), sub.next())
                .await
                .unwrap()
                .unwrap(),
            "peer data"
        );
        tokio::time::timeout(
            Duration::from_secs(5),
            rat::bacon_async(
                "unchanged_variable",
                &mut value,
                rat::VariableType::StaticOnly,
            ),
        )
        .await
        .unwrap()
        .unwrap();
        assert_eq!(value, 42);
        rat::deinit().unwrap();
    });
}

#[test]
fn coordinator_rat_and_peer_pubsub_coexist_on_isolated_ports() {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    let status = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "coordinated_rat_worker", "--nocapture"])
        .env("MINOT_TEST_RAT_COEXIST", "1")
        .env("MINOT_ZENOH_PORT", port.to_string())
        .env("MINOT_DOMAIN_ID", port.to_string())
        .status()
        .unwrap();
    assert!(status.success());
}
