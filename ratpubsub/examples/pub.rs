use std::time::Duration;

use ratpubsub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    let node = Node::create("pubber").await?;

    let msg = msg::String {
        data: "heyhio".to_owned(),
    };

    loop {
        sleep(Duration::from_secs(1)).await;
        node.publish("some_topic", &msg).await?;
    }
}
