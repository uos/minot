use log::info;
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceServer;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let server_node = Arc::new(Node::create(NodeConfig::new("server")).await?);
    let service: Arc<ServiceServer<msg::String, msg::String>> =
        ServiceServer::new(server_node.clone(), "my_cool_service".to_owned()).await?;

    let counter: Arc<Mutex<u64>> = Arc::new(Mutex::new(0));

    // syntactically kinda ugly but seems to be required to have a mutable
    // reference inside an async closure
    let service_handle = tokio::spawn(ServiceServer::start(
        service,
        Arc::new(move |msg: msg::String| {
            let counter = Arc::clone(&counter);
            async move {
                let res = if msg.data.contains("bad words") {
                    Err("No way".to_owned())
                } else {
                    Ok(msg::String {
                        data: format!("Your message was: {}", msg.data),
                    })
                };

                {
                    let mut counter = counter.lock().await;
                    *counter += 1;
                    info!("Already handled {} requests!", &counter);
                }

                res
            }
        }),
    ));

    service_handle.await?;

    Ok(())
}
