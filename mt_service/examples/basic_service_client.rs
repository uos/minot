use log::{error, info};
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceClient;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::{self, Duration};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Arc::new(Node::create(NodeConfig::new("client")).await?);
    let client: ServiceClient<msg::String, msg::String> =
        ServiceClient::new(node.clone(), "my_cool_service".to_owned()).await?;

    let mut i = 0;

    loop {
        let req = msg::String {
            data: format!("This is message number {}!", i),
        };

        info!("Sending request: {}", req.data);
        match client.request(req, Some(Duration::from_secs(10))).await {
            Ok(res) => {
                info!("Got response: {}", res.data);
            }
            Err(e) => {
                error!("Service error: {}", e);
                break;
            }
        }

        i += 1;
        time::sleep(Duration::from_secs(1)).await;
    }

    Ok(())
}
