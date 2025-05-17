use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let node = Node::create("subber".to_owned()).await?;
    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10)
        .await?;

    while let Some(msg) = subber.next().await {
        println!("Received Message: {}", &msg.data);
    }

    Ok(())
}
