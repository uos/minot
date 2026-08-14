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
