use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Node::create(NodeConfig::new("node2")).await?;
    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10, Qos::Reliable)
        .await?;
    let pubber = node
        .create_publisher::<msg::String>("/another_topic".to_owned(), Qos::Reliable)
        .await?;
    let pubber2 = node
        .create_publisher::<msg::String>("/boring_topic".to_owned(), Qos::Reliable)
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

    let shutdown = node.shutdown_token();
    loop {
        tokio::select! {
            msg = subber.next() => {
                match msg {
                    Some(msg) => {
                        println!("Got: {}", &msg.data);
                        pubber.publish(&msg_out).await.unwrap();
                    }
                    None => break,
                }
            }
            _ = shutdown.cancelled() => break,
            _ = tokio::signal::ctrl_c() => break,
        }
    }

    Ok(())
}
