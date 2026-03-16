use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

/// Demonstrates best-effort subscriber behaviour: packet drops and publisher resilience.
///
/// Runs entirely in one process — no second terminal needed.
///
/// Phases:
///   1. Publisher + BE subscriber both live. Subscriber prints received seq numbers.
///   2. Subscriber is dropped (simulates a crashed/disconnected node).
///      Publisher keeps publishing — packets are silently dropped at the coordinator.
///   3. A new BE subscriber reconnects. It starts receiving again from wherever
///      the publisher is now, proving the messages sent during the gap were lost.
///
/// Key observations:
///   - The publisher `publish()` call always returns in microseconds, even when
///     no subscriber is listening (or the subscriber is dead).
///   - Sequence numbers received in Phase 3 start well above where Phase 1 left off.
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let pub_node = Node::create(NodeConfig::new("be_drop_publisher")).await?;
    let pubber = pub_node
        .create_publisher::<msg::String>("/be_drop_topic".to_owned(), Qos::Reliable)
        .await?;

    // ── Phase 1: publisher + live subscriber ────────────────────────────────
    println!("\n=== Phase 1: publisher + BE subscriber live (3 s) ===");

    let sub_node = Node::create(NodeConfig::new("be_drop_subscriber")).await?;
    let mut subber = sub_node
        .create_subscriber::<msg::String>("/be_drop_topic".to_owned(), 8, Qos::BestEffort)
        .await?;

    let sub_handle = tokio::spawn(async move {
        while let Some(m) = subber.next().await {
            println!("  [subscriber] received: {}", m.data);
        }
        println!("  [subscriber] channel closed");
    });

    let mut seq: u64 = 0;
    let mut interval = tokio::time::interval(std::time::Duration::from_millis(400));
    let phase1_end = tokio::time::Instant::now() + std::time::Duration::from_secs(3);
    loop {
        tokio::select! {
            _ = interval.tick() => {
                seq += 1;
                let before = std::time::Instant::now();
                pubber.publish(&msg::String { data: format!("seq={seq}") }).await?;
                println!("[publisher] published seq={seq} in {:?}", before.elapsed());
                if tokio::time::Instant::now() >= phase1_end {
                    break;
                }
            }
            _ = tokio::signal::ctrl_c() => return Ok(()),
        }
    }

    // ── Phase 2: drop subscriber, keep publishing ───────────────────────────
    println!("\n=== Phase 2: subscriber dropped — publisher keeps going (4 s) ===");
    drop(sub_node); // drops Node + Subscriber, triggers disconnect on coordinator
    sub_handle.abort();

    let phase2_end = tokio::time::Instant::now() + std::time::Duration::from_secs(4);
    loop {
        tokio::select! {
            _ = interval.tick() => {
                seq += 1;
                let before = std::time::Instant::now();
                pubber.publish(&msg::String { data: format!("seq={seq}") }).await?;
                println!("[publisher] published seq={seq} in {:?} — no subscriber, packet dropped", before.elapsed());
                if tokio::time::Instant::now() >= phase2_end {
                    break;
                }
            }
            _ = tokio::signal::ctrl_c() => return Ok(()),
        }
    }

    // ── Phase 3: reconnect subscriber ──────────────────────────────────────
    println!("\n=== Phase 3: subscriber reconnects — note the seq gap (3 s) ===");

    let sub_node2 = Node::create(NodeConfig::new("be_drop_subscriber")).await?;
    let mut subber2 = sub_node2
        .create_subscriber::<msg::String>("/be_drop_topic".to_owned(), 8, Qos::BestEffort)
        .await?;

    let sub_handle2 = tokio::spawn(async move {
        while let Some(m) = subber2.next().await {
            println!(
                "  [subscriber] received: {} ← gap proves packets were dropped",
                m.data
            );
        }
    });

    let phase3_end = tokio::time::Instant::now() + std::time::Duration::from_secs(3);
    loop {
        tokio::select! {
            _ = interval.tick() => {
                seq += 1;
                let before = std::time::Instant::now();
                pubber.publish(&msg::String { data: format!("seq={seq}") }).await?;
                println!("[publisher] published seq={seq} in {:?}", before.elapsed());
                if tokio::time::Instant::now() >= phase3_end {
                    break;
                }
            }
            _ = tokio::signal::ctrl_c() => return Ok(()),
        }
    }

    sub_handle2.abort();
    println!("\nDone. Publisher never blocked or errored across all three phases.");
    Ok(())
}
