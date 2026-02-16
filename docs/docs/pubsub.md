# Native Pub/Sub

An async implementation of a basic Publish/Subscribe pattern with a ROS-like API.

!!! warning "Currently not compatible with C"

    The library for Native Publish/Subscribe is currently only available in Rust because of decisions made for minimal overhead in serialisation/deserialisation. It also not planned to support C or C++ any time soon.

While [Bagfile Query](./bagquery.md) and [Variable Sharing](./varshare.md) are the main features of Minot, this library is just a small cherry on top. The `ratpub` library uses existing networking primitives from Variable Sharing like Rats and dynamic Rules and implements a minimal Publish/Subscribe pattern on top of it.

When used in static and simple environments, it is stable enough for replacing the ROS2 Publish/Subscribe API. But it is absolutely not on par with ROS features.


!!! info "Domain ID for Network Isolation"
    
    To avoid network collisions when multiple independent Minot instances run on the same network, you can use the `MINOT_DOMAIN_ID` environment variable. This is similar to ROS2's `ROS_DOMAIN_ID` concept.

## Coordinator

Like in ROS1, the communication needs a *Master*. In this case it is our *Coordinator*, which we can run without any arguments.

Since we do not rely on ROS here, we can simply use the pip version of Minot, which is built with Ratpub and already ships with with the Coordinator.

~~~sh
pip install minot-cli
~~~

Start the Coordinator in your terminal.

~~~sh
minot-coord
~~~

!!! info

    The example uses pre-generated messages that ship with ROS2 Jazzy to already give you the types you typically use in your Node. If that is not enough, you can add your own types that implement `Serialize`, `Deserialize` and `Archive` from `rkyv`.

After adding the library to your [Cargo.toml](./installation.md#ratpub-native-publishsubscribe), you can create a publisher and a subscriber.

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

Now just run your publisher, the subscriber and the Coordinator at the same time and observe the logs.
