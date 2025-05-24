use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

fn main() -> anyhow::Result<()> {
    let frt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    let node = frt.block_on(async { Node::create("subber".to_owned()).await.unwrap() });
    let mut subber = frt.block_on(async {
        node.create_subscriber::<msg::String>("/some_topic".to_owned(), 10)
            .await
            .unwrap()
    });

    let jh = std::thread::spawn(move || {
        frt.block_on(async {
            while let Some(msg) = subber.next().await {
                println!("Received Message: {}", &msg.data);
            }
        });
    });

    jh.join().unwrap();

    Ok(())
}
