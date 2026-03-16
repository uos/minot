use log::{error, info};
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceClient;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::{self, Duration};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Arc::new(Node::create(NodeConfig::new("client2")).await?);
    let client: Arc<ServiceClient<msg::String, msg::String>> =
        Arc::new(ServiceClient::new(node.clone(), "my_cool_service".to_owned()).await?);

    let mut i = 0;
    let choices = ["🦐", "🐓", "🦈", "🐀", "🐧", "🦆", "🐌", "🦀"];

    loop {
        let req = msg::String {
            data: choices[i % choices.len()].to_owned(),
        };
        i += 1;

        info!("Sending request: {}", req.data);

        let client = client.clone();
        let req_handle = tokio::spawn(async move { client.request(req, None).await });

        // drop every 3rd request
        if i % 3 == 0 {
            req_handle.abort();
            info!("Request dropped");
            continue;
        }

        match req_handle.await.unwrap() {
            Ok(res) => {
                info!("Got response: {}", res.data);
            }
            Err(e) => {
                error!("Service error: {}", e);
                break;
            }
        }

        time::sleep(Duration::from_secs(3)).await;
    }

    Ok(())
}
