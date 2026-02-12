use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Node::create("node2".to_owned()).await?;
    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10)
        .await?;
    let pubber = node
        .create_publisher::<msg::String>("/another_topic".to_owned())
        .await?;
    let pubber2 = node
        .create_publisher::<msg::String>("/boring_topic".to_owned())
        .await?;

    let msg_out = msg::String {
        data: "sup".to_owned(),
    };

    let boring_msg = msg::String {
        data: "🙄".to_owned(),
    };

    let mut pubbing = tokio::time::interval(std::time::Duration::from_millis(4320));

    tokio::spawn(async move {
        loop {
            pubbing.tick().await;
            pubber2.publish(&boring_msg).await.unwrap();
        }
    });

    while let Some(msg) = subber.next().await {
        println!("Got: {}", &msg.data);
        pubber.publish(&msg_out).await.unwrap();
    }

    Ok(())
}
