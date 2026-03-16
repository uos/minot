use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

/// Demonstrates the runtime error when subscribing reliably to a best-effort publisher.
///
/// A reliable subscriber expects guaranteed delivery, but a best-effort publisher
/// cannot guarantee that — so the coordinator rejects the combination at registration time.
///
/// Fix: pass Qos::BestEffort to create_subscriber (see second subscriber below).
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    // BE publisher node — registers /sensor as a best-effort-published topic.
    let be_node = Node::create(NodeConfig::new("be_publisher").mode(Qos::BestEffort)).await?;
    let _pubber = be_node
        .create_publisher::<msg::String>("/sensor".to_owned(), Qos::BestEffort)
        .await?;

    // Reliable subscriber node — tries to subscribe reliably to a BE-published topic.
    let reliable_node = Node::create(NodeConfig::new("reliable_subscriber")).await?;

    let result = reliable_node
        .create_subscriber::<msg::String>("/sensor".to_owned(), 10, Qos::Reliable)
        .await;

    match &result {
        Err(e) => eprintln!("Expected error: {e}"),
        Ok(_) => eprintln!("No error — publisher was not registered yet (race). Try again."),
    }

    // Fix: use Qos::BestEffort on the subscription instead.
    let mut subber = reliable_node
        .create_subscriber::<msg::String>("/sensor".to_owned(), 10, Qos::BestEffort)
        .await?;

    println!("BE subscription succeeded. Waiting for messages...");
    while let Some(msg) = subber.next().await {
        println!("Got: {}", msg.data);
    }

    Ok(())
}
