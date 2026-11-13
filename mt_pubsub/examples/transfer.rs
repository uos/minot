//! Same-host transfer probe for `scripts/shm_threshold.py`.
//!
//! `transfer pub|sub <payload bytes> <hz> <seconds> <besteffort|reliable>`.
//! The subscriber prints `<received> <median age us> <p90 age us>` and both
//! print their CPU time as `cpu <user+system us>`.
use mt_pubsub::{Node, NodeConfig, Qos};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
struct Probe {
    sent_us: u64,
    data: Vec<u8>,
}

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

/// User and system time of this process, from `/proc/self/stat` (Linux).
fn cpu_us() -> u64 {
    let stat = std::fs::read_to_string("/proc/self/stat").unwrap_or_default();
    let fields: Vec<&str> = stat.rsplit(')').next().unwrap_or("").split(' ').collect();
    let ticks = |index: usize| fields.get(index).and_then(|f| f.parse::<u64>().ok());
    // 100 ticks per second on every common Linux.
    (ticks(12).unwrap_or(0) + ticks(13).unwrap_or(0)) * 10_000
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let (role, size, hz, seconds) = (
        args[1].as_str(),
        args[2].parse::<usize>()?,
        args[3].parse::<f64>()?,
        args[4].parse::<f64>()?,
    );
    let qos = match args[5].as_str() {
        "reliable" => Qos::Reliable,
        _ => Qos::BestEffort,
    };
    let node =
        Node::create(NodeConfig::new(format!("transfer_{role}")).mode(Qos::BestEffort)).await?;
    let topic = "/transfer".to_owned();
    if role == "pub" {
        let publisher = node.create_publisher::<Probe>(topic, qos).await?;
        publisher
            .wait_for_subscribers(Duration::from_secs(10))
            .await?;
        // The shared memory link follows the connection.
        tokio::time::sleep(Duration::from_millis(300)).await;
        let mut tick = tokio::time::interval(Duration::from_secs_f64(1.0 / hz));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut probe = Probe {
            sent_us: 0,
            data: vec![7; size],
        };
        let (start, busy) = (Instant::now(), cpu_us());
        while start.elapsed().as_secs_f64() < seconds {
            tick.tick().await;
            probe.sent_us = now_us();
            let _ = publisher.publish(&probe).await;
        }
        println!("cpu {}", cpu_us() - busy);
    } else {
        let mut subscriber = node.create_subscriber::<Probe>(topic, 64, qos).await?;
        let mut ages = Vec::new();
        let mut busy = 0;
        let mut end = Instant::now() + Duration::from_secs(30);
        while let Ok(Some(message)) =
            tokio::time::timeout_at(end.into(), subscriber.next_archived()).await
        {
            if ages.is_empty() {
                busy = cpu_us();
                end = Instant::now() + Duration::from_secs_f64(seconds - 0.5);
            }
            ages.push(now_us().saturating_sub(message.archived().sent_us.to_native()));
        }
        let cpu = cpu_us() - busy;
        ages.sort_unstable();
        let at = |q: f64| {
            ages.get((ages.len() as f64 * q) as usize)
                .copied()
                .unwrap_or(0)
        };
        println!("{} {} {}", ages.len(), at(0.5), at(0.9));
        println!("cpu {cpu}");
    }
    Ok(())
}
