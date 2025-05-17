use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let node = Node::create("pubber".to_owned()).await?;
    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned())
        .await?;

    let msg = msg::String {
        data: "heyhio".to_owned(),
    };

    loop {
        sleep(std::time::Duration::from_secs(1)).await;
        pubber.publish(&msg).await?;
    }
}
