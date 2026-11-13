//! Link outage probe for `scripts/peer_blackout.sh`.
//!
//! `blackout pub` publishes three topics; `blackout sub` prints one line per
//! received sample: `<unix ms> <topic> <sequence> <age ms>`.
use mt_pubsub::{Node, NodeConfig, Qos};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
struct Probe {
    sequence: u64,
    sent_ms: u64,
    data: Vec<u8>,
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
}

/// Topic, QoS, payload bytes, period.
const TOPICS: [(&str, Qos, usize, u64); 3] = [
    ("small", Qos::BestEffort, 64, 20),
    ("large", Qos::BestEffort, 500_000, 100),
    ("try", Qos::TryReliable, 64, 50),
];

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let role = std::env::args().nth(1).unwrap_or_default();
    let node =
        Node::create(NodeConfig::new(format!("blackout_{role}")).mode(Qos::BestEffort)).await?;
    let mut tasks = tokio::task::JoinSet::new();
    let env = |name: &str| {
        std::env::var(name)
            .ok()
            .and_then(|value| value.parse().ok())
    };
    for (topic, qos, mut size, mut period) in TOPICS {
        if topic == "large" {
            // `BLACKOUT_LARGE_BYTES` and `BLACKOUT_LARGE_MS` change the large topic.
            size = env("BLACKOUT_LARGE_BYTES").unwrap_or(size);
            period = env("BLACKOUT_LARGE_MS").unwrap_or(period as usize) as u64;
        }
        let name = format!("/blackout/{topic}");
        if role == "pub" {
            let publisher = node.create_publisher::<Probe>(name, qos).await?;
            tasks.spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(period));
                tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                let mut probe = Probe {
                    sequence: 0,
                    sent_ms: 0,
                    data: vec![7; size],
                };
                loop {
                    tick.tick().await;
                    probe.sequence += 1;
                    probe.sent_ms = now_ms();
                    let _ = publisher.publish(&probe).await;
                }
            });
        } else {
            let mut subscriber = node.create_subscriber::<Probe>(name, 64, qos).await?;
            tasks.spawn(async move {
                while let Some(message) = subscriber.next_archived().await {
                    let probe = message.archived();
                    let now = now_ms();
                    let age = now.saturating_sub(probe.sent_ms.to_native());
                    println!("{now} {topic} {} {age}", probe.sequence.to_native());
                }
                println!("{} {topic} closed 0", now_ms());
            });
        }
    }
    node.shutdown_token().cancelled().await;
    Ok(())
}
