use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let node = Node::create("node1".to_owned()).await?;
    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned())
        .await?;
    let mut subber = node
        .create_subscriber::<msg::String>("/another_topic".to_owned(), 10)
        .await?;
    let mut subber2 = node
        .create_subscriber::<msg::String>("/boring_topic".to_owned(), 10)
        .await?;
    let mut useless_subber = node
        .create_subscriber::<msg::String>("/nothing_here".to_owned(), 10)
        .await?;

    let msg = msg::String {
        data: "heyhio".to_owned(),
    };

    tokio::spawn(async move {
        while let Some(_) = useless_subber.next().await {
            log::error!("ehh.. no?");
        }
    });

    let mut pubbing = tokio::time::interval(std::time::Duration::from_secs(1));
    loop {
        tokio::select! {
            _ = pubbing.tick() => {
                pubber.publish(&msg).await?;
            }
            msg_in = subber.next() => {
                match msg_in {
                    Some(msg_in) => {
                        println!("Got: {}", msg_in.data);
                    },
                    None => {
                        break;
                    },
                }
            }
            msg_in = subber2.next() => {
                match msg_in {
                    Some(msg_in) => {
                        println!("Got: {}", msg_in.data);
                    },
                    None => {
                        break;
                    },
                }
            }
        }
    }

    Ok(())
}
