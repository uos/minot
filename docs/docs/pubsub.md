# Native Pub/Sub

An async implementation of a basic Publish/Subscribe pattern with a ROS-like API.

!!! warning "Currently not compatible with C"

    The library for Native Publish/Subscribe is currently only available in Rust because of decisions made for minimal overhead in serialisation/deserialisation. It also not planned to support C or C++ any time soon.

While [Bagfile Query](./bagquery.md) and [Variable Sharing](./varshare.md) are the main features of Minot, this library is just a small cherry on top. The `ratpub` library uses existing networking primitives from Variable Sharing like Rats and dynamic Rules and implements a minimal Publish/Subscribe pattern on top of it.

When used in static and simple environments (no lifecycle node, assuming stable network and more), it is stable enough for replacing the ROS2 Publish/Subscribe API. But it is absolutely not on par with ROS features.

After adding the library to your [Cargo.toml](./installation.md#ratpub-native-publishsubscribe), you can create a Publisher and a Subscriber. Like in ROS1, the communication needs a *Master*. In this case it is our Coordinator, which we can run [standalone](./installation.md#standalone-coordinator) without any arguments.

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


In contrast to Variable Sharing, the Publisher does not care about syncing with the Subscriber. But like most abstractions, this comes at a cost. Here it is setting the capacity of the queue length for the Subscriber - like in ROS.

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

    - If a Publisher dies or disconnects in an existing Pub/Sub connection the Subscriber will never get messages on this topic again, even when the Publisher comes back or other ones still publish on that topic.
    - Loosing connections to the Coordinator at runtime will result in an error. `ratpub` is therefore not really reliable in unrealiable networks like WIFI.
    - Nodes are always talking to the Coordinator, which is a good feature for reconfigurable topics but also adds latency. The plan is to add static and dynamic topics, where both have the same features but both have better latency for their respective use case.


