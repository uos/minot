# Native Pub/Sub

An async implementation of a basic Publish/Subscribe pattern with a ROS-like API.

!!! warning "Currently not compatible with C"

    The library for Native Publish/Subscribe is currently only available in Rust because of decisions made for minimal overhead in serialisation/deserialisation. It also not planned to support C or C++ any time soon.

While [Bagfile Query](./bagquery.md) and [Variable Sharing](./varshare.md) are the main features of Minot, this library is just a small cherry on top. The `ratpub` library uses existing networking primitives from Variable Sharing like Rats and dynamic Rules and implements a minimal Publish/Subscribe pattern on top of it.

When used in static and simple environments (no lifecycle node, assuming stable network and more), it is stable enough for replacing the ROS2 Publish/Subscribe API. But it is absolutely not on par with ROS features.

After adding the library to your [Cargo.toml](./installation.md#ratpub-native-publishsubscribe), you can create a publisher and a subscriber. Like in ROS1, the communication needs a *Master*. In this case it is our Coordinator, which we can run [standalone](./installation.md#standalone-coordinator) without any arguments.

## Domain ID for Network Isolation

To avoid network collisions when multiple independent Minot instances run on the same network, you can use the `MINOT_DOMAIN_ID` environment variable. This is similar to ROS2's `ROS_DOMAIN_ID` concept.

The domain ID is included in the discovery protocol, so nodes from different domains will ignore each other even though they share the same UDP port (6594). This approach doesn't reserve multiple ports on your system.

~~~sh title="Start the Coordinator with Domain ID"
# Default domain (0)
minot-coord

# Or specify a domain ID
MINOT_DOMAIN_ID=5 minot-coord
~~~

**All nodes and the coordinator in the same network must use the same domain ID:**

~~~sh title="publisher.rs with domain ID"
MINOT_DOMAIN_ID=5 cargo run --bin publisher
~~~

~~~sh title="subscriber.rs with domain ID"
MINOT_DOMAIN_ID=5 cargo run --bin subscriber
~~~

!!! info "Domain ID Range"

    Domain IDs from 0-99 are supported. Invalid or out-of-range values default to domain 0.

~~~sh title="Start the Coordinator in your Shell"
minot-coord
~~~

The example uses pre-generated messages that ship with ROS2 Jazzy to already give you the types you typically use in your Node. If that is not enough, you can add your own types that implement `Serialize`, `Deserialize` and `Archive` from `rkyv`.

~~~rust title="publisher.rs"
use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // env_logger::init(); // some useful output when running: RUST_LOG=info cargo run 

    // Resolves when connected to the Coordinator
    let node = Node::create("my_node_pub".to_owned()).await?;     

    let pubber = node
        .create_publisher::<msg::String>("/some_topic".to_owned())
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
use ratpub::Node;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // env_logger::init();

    let node = Node::create("my_node_sub".to_owned()).await?;

    let mut subber = node
        .create_subscriber::<msg::String>("/some_topic".to_owned(), 10)
        .await?;

    while let Some(msg) = subber.next().await {
        println!("Received Message: {}", &msg.data);
    }


    Ok(())
}
~~~

Mind that we don't need to `spin_once` here thanks to async like in other Rust ROS libraries.

!!! warning "Unstable Networks"

    In the current state, `ratpub` should only be used in stable networks. There are still use cases that break your expectations. These problems are planned to get fixed though.

    Examples

    - If a publisher dies or disconnects in an existing Pub/Sub connection the subscriber will never get messages on this topic again, even when the publisher comes back or other ones still publish on that topic.
    - Loosing connections to the Coordinator at runtime will result in an error. `ratpub` is therefore not really reliable in unrealiable networks like Wi-Fi.
    - Nodes are always talking to the Coordinator, which is a good feature for reconfigurable topics but also adds latency. The plan is to add static and dynamic topics, where both have the same features but both have better latency for their respective use case.


