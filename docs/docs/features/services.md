# Services

Services implement a Request/Reply (RPC) pattern, compatible with ROS2 semantics but running natively over the Minot network stack.

!!! tip "Golang Support"

    The `mt_service` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

Built on top of `mt_pubsub`, the `mt_service` crate provides robust request-reply communication with internal sequence tracking to ensure that responses are correctly matched to their corresponding requests.

After adding `mt_service` to your `Cargo.toml`, you can implement a service server and client.

## Server

The server handles incoming requests and provides a response asynchronously.

~~~rust title="service_server.rs"
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceServer;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Node creation (auto-starts coordinator if needed)
    let node = Arc::new(Node::create(NodeConfig::new("service_server")).await?);
    
    let server = ServiceServer::<msg::String, msg::String>::new(
        node.clone(), 
        "/my_service".to_owned()
    ).await?;

    ServiceServer::start(
        server,
        Arc::new(|req: msg::String| async move {
            println!("Got request: {}", req.data);
            Ok(msg::String {
                data: format!("Response to {}", req.data),
            })
        }),
    ).await;

    Ok(())
}
~~~

## Client

The client sends a request and waits for the response.

~~~rust title="service_client.rs"
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceClient;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::Duration;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Arc::new(Node::create(NodeConfig::new("service_client")).await?);
    
    let client = ServiceClient::<msg::String, msg::String>::new(
        node.clone(), 
        "/my_service".to_owned()
    ).await?;

    let request = msg::String {
        data: "Hello from client".to_owned(),
    };

    match client.request(request, Some(Duration::from_secs(5))).await {
        Ok(response) => println!("Got response: {}", response.data),
        Err(e) => eprintln!("Service call failed: {}", e),
    }

    Ok(())
}
~~~
