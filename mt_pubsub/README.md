# mt_pubsub

A simple and deterministic Pub/Sub implementation with Minot primitives.

Part of the [minot](https://github.com/uos/minot) family of crates.

[![Latest version](https://img.shields.io/crates/v/mt_pubsub.svg)](https://crates.io/crates/mt_pubsub)
![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Apache](https://img.shields.io/badge/license-Apache-blue.svg)


~~~toml title="Cargo.toml"
[dependencies]
mt_pubsub = "0.9.0"
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
cargo run --bin minot -- coord
```

Then run the publisher and subscriber in separate terminals:
```bash
# Terminal 2
cargo run --example pub

# Terminal 3
cargo run --example sub
```


If you have multiple Minot networks on the same physical network, use the `MINOT_DOMAIN_ID` environment variable to prevent them from connecting to each other.

## Zenoh coordinator port

`MINOT_ZENOH_PORT` selects the coordinator TCP port (default: `7447`). Set it
before starting Minot and every node that shares that coordinator. It controls
the router listener, local-only/unicast client endpoint, and local reachability
probe. Explicit `MINOT_COORD_ADDR` endpoints keep their specified port. Invalid
or zero port values warn and fall back to `7447`.

To coexist with a ROS Zenoh router already using port 7447:

```bash
export MINOT_ZENOH_PORT=17447
minot --local-only coord
```

Set the same variable in the shells or containers running your Minot nodes.
Local-only mode restricts sessions to loopback; for a remote coordinator use an
explicit endpoint such as `MINOT_COORD_ADDR=tcp/192.168.1.10:17447`.

Embedded coordinator election uses a per-user, per-port file lock. Concurrent
processes sharing the same temporary filesystem elect one coordinator; the
other processes retry registration while it starts. Containers with separate
filesystems and machines on different hosts do not share that election. For
those deployments designate one coordinator and use `CoordMode::External` on
nodes that must not auto-start another. Peer/gossip transport discovery does
not elect a Minot coordinator.

## Delivery modes

`Qos` combines `Reliability` and `Durability`. Reliability also controls failure
handling and congestion behavior.

| | Wire | Congestion | Peer monitoring | On loss | Send dispatch | Durability |
|---|---|---|---|---|---|---|
| `Reliable` | reliable, acknowledged per sample | `Block` | yes | torpedoes the run | inline, unbounded retry | volatile |
| `TryReliable` | reliable, not acknowledged | `Block` | no | rest keeps running | spawned, bounded by budget | volatile |
| `BestEffort` | best-effort | `Drop` | no | rest keeps running | inline, single attempt | volatile |
| `.transient_local()` | preserves base mode | preserves base mode | preserves base mode | preserves base mode | preserves base mode | replay latest sample to late subscribers |

`Reliable` is the default and the strict one: peers heartbeat-monitor each other, and a
node that stops answering tears the whole run down. That is what you want for a
deterministic replay where a missing participant invalidates the result.

`TryReliable` keeps reliable delivery and ordering but drops the fatal part. It behaves
like a ROS 2 DDS `RELIABLE` subscriber: a write that cannot land within its deadline
reports its own deadline failure while peers stay connected. Sends
are dispatched off the caller's thread, so a slow or roaming link cannot wedge a
publisher's loop. There is no end-to-end acknowledgement; a congested link blocks
the send up to the budget instead of dropping it.

`BestEffort` tries once and moves on.

Transient-local durability combines with every reliability mode. It retains the latest
sample and replays it to late subscribers. It is disabled by default.

```rust
let publisher = node
    .create_publisher::<Transform>("/tf_static".to_owned(), Qos::Reliable.transient_local())
    .await?;
```

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
(default 3000) bounds how long one sample may wait for a congested link.

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

## Camera image conversion

The default `image` feature converts `sensor_msgs::msg::Image` and
`CompressedImage` messages into `image::DynamicImage`:

```toml
mt_pubsub = "0.13"
```

```rust
use mt_pubsub::{ToImage, ImageMessageExt, image::{
    image_from_dynamic, compressed_image_from_dynamic, ImageFormat, Time,
}};

let pixels = message.to_image()?; // Image or CompressedImage
let raw = image_from_dynamic(&pixels)?
    .with_header(message.header.clone());
let png = compressed_image_from_dynamic(&pixels, ImageFormat::Png)?
    .with_frame_id("camera_optical_frame")
    .with_stamp(Time { sec: 42, nanosec: 123 });
```

The helper copies raw pixels into an owned image, removes row padding, respects
16-bit byte order, and converts BGR/BGRA to RGB/RGBA. Supported raw encodings are
`mono8`, `rgb8`, `bgr8`, `rgba8`, `bgra8`, their `16` variants, `8UC1`, and
`16UC1`. Grayscale and depth samples retain their original values without scaling.
JPEG and PNG compressed payloads are decoded by their signatures, including
16-bit PNG. Additional codecs can be enabled through an application dependency
on the `image` crate. Bayer, YUV, floating-point depth, and `compressedDepth`
transport need specialized processing and return errors. Malformed buffers also
return errors.

Encoding produces tightly packed raw grayscale/RGB/RGBA messages at 8 or 16
bits, or JPEG/PNG compressed messages. Headers default to zero/empty. Use the fluent
`with_header`, `with_frame_id`, and `with_stamp` setters before publishing.
Setters apply in order; `with_header` replaces the complete header. PNG preserves supported
samples and alpha; JPEG is lossy and accepts 8-bit grayscale/RGB. Convert other
color types explicitly before JPEG encoding. Unsupported color types return an
error rather than silently losing precision or alpha.

Archived references from `next_archived().archived()` can be decoded using
`image::archived_image_to_image` or `image::archived_compressed_image_to_image`,
without first deserializing the message payload.

For use without `mt_pubsub`, import `mt_net::image::ToImage`; `mt_net` also
enables `image` by default. Use `default-features = false` to omit image codecs
when only forwarding messages.
