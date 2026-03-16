use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

/// Best-effort subscriber example.
///
/// A best-effort subscriber tells the coordinator that it is allowed to be skipped
/// if it is slow or unresponsive. The publisher will fire-and-forget to this subscriber
/// instead of blocking until delivery is confirmed.
///
/// Required when subscribing to a topic published by a best-effort node.
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    // The node itself can be reliable — BE is per-subscription, not per-node.
    let node = Node::create(NodeConfig::new("reliable_node_with_be_sub")).await?;

    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10, Qos::BestEffort)
        .await?;

    while let Some(msg) = subber.next().await {
        println!("Got: {}", msg.data);
    }

    Ok(())
}
