# Pub/Sub

An async implementation of a basic Publish/Subscribe pattern with a ROS-like API, but running natively over the Minot network stack.

!!! tip "Golang Support"

    The `mt_pubsub` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

While [Bagfile Query](./bagquery.md) and [Variable Sharing](./varshare.md) are the main features of Minot, this library is just a small cherry on top. The `mt_pubsub` library uses existing networking primitives from Minot (like dynamic Rules and Ships) and implements a minimal Publish/Subscribe pattern on top of it.

When used in static and simple environments, it is stable enough for replacing the ROS2 Publish/Subscribe API. But it is absolutely not on par with ROS features.


!!! info "Domain ID for Network Isolation"
    
    To avoid network collisions when multiple independent Minot instances run on the same network, you can use the `MINOT_DOMAIN_ID` environment variable. This is similar to ROS2's `ROS_DOMAIN_ID` concept.

## Automatic Coordinator

Like in ROS1, the communication technically needs a *Master* to distribute routes. In our case, it is the *Coordinator*. 

**However**, you do not need to start it manually! When a Minot node starts up (e.g. your publisher or subscriber), it will attempt to find a coordinator on the network. If it times out, the node will **automatically spin up an embedded coordinator** in the background.

This means you can just run your nodes directly without worrying about network setup!

!!! info

    The examples below use pre-generated messages that ship with ROS2 Jazzy to already give you the types you typically use in your Node. If that is not enough, you can add your own types that implement `Serialize`, `Deserialize` and `Archive` from `rkyv`.

## Examples

After adding the library to your `Cargo.toml`, you can create a publisher and a subscriber.

~~~rust title="publisher.rs"
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Resolves when connected to a Coordinator (or automatically starts one)
    let node = Node::create(NodeConfig::new("my_node_pub")).await?;     

    // Qos can be Reliable or BestEffort
    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned(), Qos::Reliable)
        .await?;

    let msg = msg::String {
        data: "Hello World 🖖".to_owned(),
    };

    loop {
        pubber.publish(&msg).await?;
        sleep(Duration::from_secs(1)).await;
    }
}
~~~

!!! info

    Nodes will automatically detect the Coordinator and do not need any setup. Like the other networking stack from Minot, you just need to ensure that each Node that you want to connect with each other is reachable in the network.


In contrast to Variable Sharing, the publisher does not care about syncing with the subscriber. But like most abstractions, this comes at a cost. Here it is setting the capacity of the queue length for the subscriber - like in ROS.

~~~rust title="subscriber.rs"
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let node = Node::create(NodeConfig::new("my_node_sub")).await?;

    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10, Qos::Reliable)
        .await?;

    while let Some(msg) = subber.next().await {
        println!("Received Message: {}", &msg.data);
    }


    Ok(())
}
~~~

Mind that we don't need to `spin_once` here thanks to async like in other Rust ROS libraries.
