use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

/// Best-effort node example.
///
/// A best-effort node will NOT cause other nodes to be torpedoed if it crashes.
/// Use this for non-critical publishers (e.g. bag playback, optional data sources).
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Node::create(NodeConfig::new("be_publisher").mode(Qos::BestEffort)).await?;

    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned(), Qos::BestEffort)
        .await?;

    let msg = msg::String {
        data: "hello from a best-effort node".to_owned(),
    };

    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    loop {
        interval.tick().await;
        pubber.publish(&msg).await?;
        println!("Published");
    }
}
