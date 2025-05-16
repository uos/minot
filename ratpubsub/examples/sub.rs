use ratpubsub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let node = Node::create("subber").await?;
    let mut rx = node.subscribe::<msg::String>("some_topic", 10)?;

    while let Some(msg) = rx.recv().await {
        println!("Received Message: {}", &msg.data);
    }

    Ok(())
}
