# Services

Services implement a Request/Reply (RPC) pattern, compatible with ROS2 semantics but running natively over Minot's [Pub/Sub](./pubsub.md).

!!! tip "Golang Support"

    The `mt_service` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

`mt_service` sends each request with the client's ID and a sequence number, so every response reaches the request that asked for it, even when one client has several requests in flight. Both sides use reliable topics.

After adding `mt_service` to your `Cargo.toml`, you can implement a service server and client. The examples use `std_msgs/String` from `ros2_interfaces_jazzy_rkyv`.

## Server

The callback gets the request and returns `Ok(response)` or `Err(message)`. An error reaches the client as `Err(message)` from its request.

~~~rust title="service_server.rs"
use mt_pubsub::{Node, NodeConfig};
use mt_service::ServiceServer;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Arc::new(Node::create(NodeConfig::new("service_server")).await?);

    let server = ServiceServer::<msg::String, msg::String>::new(
        node.clone(),
        "/my_service".to_owned(),
    )
    .await?;

    // Runs until the node shuts down.
    ServiceServer::start(
        server,
        Arc::new(|req: msg::String| async move {
            if req.data.is_empty() {
                return Err("Empty request".to_owned());
            }
            Ok(msg::String {
                data: format!("Response to {}", req.data),
            })
        }),
    )
    .await;

    Ok(())
}
~~~

### Concurrent requests

`ServiceServer::start` answers one request per client at a time, in the order they arrived. A slow request delays everything the same client queued behind it.

`ServiceServer::start_concurrent` runs up to a given number of one client's requests at once:

~~~rust
ServiceServer::start_concurrent(server, callback, 8).await;
~~~

Responses may then arrive in a different order than the requests were sent, and two requests of the same client may run at the same time. Keep requests that change shared state step by step (stage, write, commit) on `start`, or send them one at a time. Read-only requests are safe either way.

## Client

`request` sends a request and waits for its response. It returns `Err(message)` when the server reports an error, when sending fails, or when the timeout passes. With `None` it waits as long as it takes.

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
        "/my_service".to_owned(),
    )
    .await?;

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

A client can be shared. Put it in an `Arc` and call `request` from several tasks at once. Each call waits for its own response. A request that times out frees its slot, and a response that arrives later is dropped.

More examples are in `mt_service/examples/` in the Minot repository.
