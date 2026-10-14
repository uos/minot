# Pub/Sub

An async Publish/Subscribe API running directly over Zenoh peers by default.

!!! tip "Golang Support"

    The `mt_pubsub` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

While [Bagfile Query](./bagquery.md) and [Variable Sharing](./varshare.md) are the main features of Minot, this library is just a small cherry on top. The `mt_pubsub` library uses existing networking primitives from Minot (like dynamic Rules and Ships) and implements a minimal Publish/Subscribe pattern on top of it.

When used in static and simple environments, it is stable enough for replacing the ROS2 Publish/Subscribe API. But it is absolutely not on par with ROS features.


!!! info "Domain ID for Network Isolation"
    
    To avoid network collisions when multiple independent Minot instances run on the same network, you can use the `MINOT_DOMAIN_ID` environment variable. This is similar to ROS2's `ROS_DOMAIN_ID` concept.

## Peer Discovery

`NodeConfig::new` selects `CoordMode::Peer`. Nodes declare typed topics directly
in Zenoh without starting a Minot coordinator or acquiring a lock file.
Multicast scouting and single-hop gossip connect peers automatically. `MINOT_ZENOH_PORT`
(default `7447`) selects the peer UDP scouting port and the existing coordinator
TCP port; use a different value on all Minot nodes to coexist with ROS Zenoh.
Peer listeners use OS-assigned TCP ports unless explicitly configured.

For Docker or routed networks without multicast, supply reachable peers through
`MINOT_PEERS=tcp/seed:17447` (comma-separated endpoints), or use
`NodeConfig::peer_endpoints` and `NodeConfig::listen_endpoint`. Explicit peers
disable multicast; gossip remains enabled. Bootstrap nodes are ordinary peers.
They must advertise reachable listener addresses for mesh expansion.

Discovery is asynchronous. Use
`publisher.wait_for_subscribers(Duration::from_secs(5)).await?` when at least one
recipient must be discovered before publishing the first sample. For a known
startup group, `wait_for_subscriber_count(count, timeout)` waits for the
required number of receiving endpoints. Startup QoS checks and counts use the
peers observed so far if a discovery query times out during mesh expansion;
Reliable delivery still requires a complete subscriber snapshot and all ACKs.
Local-only peers choose one connection initiator per pair to avoid duplicate
connection attempts.

## Coordinated Compatibility

`mt_rat` variable synchronization and bag-query coordination retain their
coordinator protocol. Pub/sub applications can explicitly select
`CoordMode::AutoStart`, `Start`, or `External` to use the previous topic routing.
Peer and coordinated pub/sub endpoints use different wire protocols; select
the same mode on both sides of a topic.

The coordinator's listener and local startup probe honor `MINOT_ZENOH_PORT`.
Explicit `MINOT_COORD_ADDR` keeps its own port. Its per-user/per-port lock still
requires a shared filesystem; use one designated coordinator for coordinated
applications across separate containers.

Peer `Reliable` delivery acknowledges subscriber queue acceptance. Missing
acknowledgements or loss of a fatal reliable node cancels the peer run.
Transient-local writers provide retained samples directly to late subscribers.
Peer loss detection uses Zenoh liveliness; coordinator heartbeat/reconnect
settings continue to apply to coordinated modes. The peer implementation uses
owned Zenoh buffers; explicit Minot SHM pool controls remain on the coordinated
path.

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
    // Opens a coordinator-free Zenoh peer session
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

    Nodes discover each other through multicast or bootstrap peers and gossip. Ensure that their advertised listener addresses are reachable.


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
