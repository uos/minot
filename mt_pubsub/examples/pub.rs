use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Node::create(NodeConfig::new("node1")).await?;
    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned(), Qos::Reliable)
        .await?;
    let mut subber = node
        .create_subscriber::<msg::String>("/another_topic".to_owned(), 10, mt_pubsub::Qos::Reliable)
        .await?;
    let mut subber2 = node
        .create_subscriber::<msg::String>("/boring_topic".to_owned(), 10, mt_pubsub::Qos::Reliable)
        .await?;
    let mut useless_subber = node
        .create_subscriber::<msg::String>("/nothing_here".to_owned(), 10, mt_pubsub::Qos::Reliable)
        .await?;

    let msg = msg::String {
        data: "heyhio".to_owned(),
    };

    tokio::spawn(async move {
        while useless_subber.next().await.is_some() {
            log::error!("ehh.. no?");
        }
    });

    let shutdown = node.shutdown_token();
    let mut pubbing = tokio::time::interval(std::time::Duration::from_secs(1));
    loop {
        tokio::select! {
            _ = pubbing.tick() => {
                // Spawn so a slow/hanging publish doesn't block ctrl-c or shutdown.
                let pubber = pubber.clone();
                let msg = msg.clone();
                tokio::spawn(async move {
                    if let Err(e) = pubber.publish(&msg).await {
                        log::error!("Publish failed: {e}");
                    }
                });
            }
            msg_in = subber.next() => {
                match msg_in {
                    Some(msg_in) => println!("Got: {}", msg_in.data),
                    None => break,
                }
            }
            msg_in = subber2.next() => {
                match msg_in {
                    Some(msg_in) => println!("Got: {}", msg_in.data),
                    None => break,
                }
            }
            _ = shutdown.cancelled() => break,
            _ = tokio::signal::ctrl_c() => break,
        }
    }

    Ok(())
}
