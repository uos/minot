# mt_pubsub

A simple and deterministic Pub/Sub implementation with Minot primitives.

Part of the [minot](https://github.com/uos/minot) family of crates.

[![Latest version](https://img.shields.io/crates/v/mt_pubsub.svg)](https://crates.io/crates/mt_pubsub)
![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Apache](https://img.shields.io/badge/license-Apache-blue.svg)


~~~toml title="Cargo.toml"
[dependencies]
mt_pubsub = "0.8.0"
tokio = { version = "1.49", features = ["full"] }
~~~

Since you probably want to use existing ROS2 message definitions, you can also add the following crate which is auto-generated from the Jazzy release.

~~~toml title="Cargo.toml"
[dependencies]
ros2-interfaces-jazzy-rkyv = { version = "0.0.5", features = [
  "std_msgs", # add more here
] }
~~~

Learn more on how to use it in your Code by visiting the [Minot docs](https://uos.github.io/minot/pubsub.html).


## Running the Examples

First, start the coordinator:
```bash
cargo run --bin minot-coord
```

Then run the publisher and subscriber in separate terminals:
```bash
# Terminal 2
cargo run --example pub

# Terminal 3
cargo run --example sub
```


If you have multiple Minot networks on the same physical network, use the `MINOT_DOMAIN_ID` environment variable to prevent them from connecting to each other.

## Delivery modes

`Qos` sets a node's delivery mode. It encodes two independent things, and it is worth
keeping them apart: how Zenoh carries the bytes, and what the rest of the system does
when the node stops answering.

| | Wire | Congestion | Peer monitoring | On loss | Send dispatch |
|---|---|---|---|---|---|
| `Reliable` | reliable | `Block` | yes | torpedoes the run | inline, unbounded retry |
| `TryReliable` | reliable | `BlockFirst` | no | rest keeps running | spawned, retry within budget |
| `BestEffort` | best-effort | `Drop` | no | rest keeps running | spawned, single attempt |

`Reliable` is the default and the strict one: peers heartbeat-monitor each other, and a
node that stops answering tears the whole run down. That is what you want for a
deterministic replay where a missing participant invalidates the result.

`TryReliable` keeps reliable delivery and ordering but drops the fatal part. It behaves
like a ROS 2 DDS `RELIABLE` subscriber: a write that cannot land within its deadline
fails on its own instead of taking the participant down, and no peer is torpedoed. Sends
are dispatched off the caller's thread, so a slow or roaming link cannot wedge a
publisher's loop.

`BestEffort` tries once and moves on.

Note that reliable wire delivery is close to free over TCP, which already retransmits and
orders. The meaningful difference between `Reliable` and `TryReliable` is the failure
policy, not the wire.

Set the mode on the node:

```rust
use mt_pubsub::{Node, NodeConfig, Qos};

let node = Node::create(NodeConfig::new("viewer").mode(Qos::TryReliable)).await?;
```

Or per topic, which overrides the node's mode for that subscription only:

```rust
let points = node
    .create_subscriber::<PointCloud2>("/ouster/points".to_owned(), 10, Qos::TryReliable)
    .await?;
```

A best-effort publisher drops samples, so a subscriber that expects delivery
(`Reliable` or `TryReliable`) is rejected at registration.
Pair a best-effort publisher with best-effort subscribers.

The `TryReliable` budget is tunable in `mt_sea`: `TRY_RELIABLE_SEND_BUDGET_MS`
(default 3000) bounds one delivery, `TRY_RELIABLE_ATTEMPT_TIMEOUT_MS` (500) bounds a
single attempt inside it, and `TRY_RELIABLE_RETRY_BACKOFF_MS` (50) spaces the retries.

## Shared memory

The `shm` feature uses Zenoh shared memory for same-device messages of at least 1 MiB.
SHM capability is negotiated per Zenoh connection. If either local process has SHM disabled or
unavailable, SHM-backed messages are carried as ordinary network bytes on that connection.
Enabling SHM therefore remains compatible with peers built or started without SHM.

Defaults can be changed with:

- `MINOT_SHM_SIZE` — initial pool size in bytes (default: 16 MiB)
- `MINOT_SHM_THRESHOLD` — minimum message size attempted through SHM (default: 1 MiB)
- `MINOT_SHM_MAX_MESSAGE_SIZE` — largest message attempted through SHM (default: 64 MiB)
- `MINOT_SHM_ALLOCATION_TIMEOUT_MS` — allocation deadline before fallback (default: 250 ms)
- `MINOT_SHM_STATS_INTERVAL_SECS` — interval for non-empty SHM/non-SHM traffic summaries
  from the `minot` binary (default: 30 seconds)
- `MINOT_SHM_DISABLED=1` — always use regular Zenoh transport

The `minot` binary exposes the same controls as `--shm-size`, `--shm-threshold`,
`--shm-max-message-size`, `--shm-allocation-timeout-ms`, and `--no-shm`.

Run the reliability suite and manual benchmark with:

```bash
cargo test -p mt_pubsub --test shm_reliability --features shm
cargo test -p mt_pubsub --test shm_reliability --features shm --release \
  shm_large_packet_benchmark -- --exact --ignored --nocapture
```

## Borrowed archived messages

`Subscriber::next()` returns an owned, deserialized message. Code that can work with
rkyv's archived representation can avoid that conversion with `next_archived()`:

```rust
if let Some(message) = subscriber.next_archived().await {
    let archived = message.archived();
    // Read archived fields while `message` keeps the backing buffer alive.
}
```

Call `message.deserialize()` when an existing API still requires an owned value.
