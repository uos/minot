# mt_pubsub

Typed Pub/Sub over Zenoh peers, with optional coordinated Minot routing.

Part of the [minot](https://minot.steado.tech) family of crates.

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

No coordinator is needed for the default peer mode.

Then run the publisher and subscriber in separate terminals:
```bash
# Terminal 1
cargo run --example pub

# Terminal 2
cargo run --example sub
```


If you have multiple Minot networks on the same physical network, use the `MINOT_DOMAIN_ID` environment variable to isolate their application topics.

## Minimal dependencies

For TCP pub/sub without image conversion codecs or Zenoh's additional transports:

```toml
mt_pubsub = { version = "0.14.0", default-features = false }
```

TCP pub/sub retains multicast scouting, gossip discovery, reliability, and liveliness.
The `image` feature adds JPEG/PNG conversion helpers; raw image messages remain available
without it. The `zenoh-defaults` feature restores Zenoh's authentication, compression,
and additional transports (including TLS, QUIC, UDP data transport, and WebSockets).
Both features remain enabled by default for existing library consumers. `mt_service`
and `mt_rat` also offer `zenoh-defaults`, and can use TCP with default features disabled.
The Minot CLI keeps the full transport set in its default feature configuration.
Shared-memory support is selected independently with `shm`.

## Peer networking and coordinator compatibility

`NodeConfig::new` now selects `CoordMode::Peer`. Publishers and subscribers
match through Zenoh directly; no coordinator startup or lock file is involved.
`mt_rat` synchronization, rules, and bag-query coordination still use the
existing coordinator protocol. Coordinated and peer topic protocols are
separate: choose the same mode on all endpoints of a topic. Old `mt_pubsub`
applications can retain their existing routing by selecting `AutoStart`,
`Start`, or `External` explicitly.

Peers listen on OS-assigned TCP ports and discover each other through multicast
and single-hop gossip. `MINOT_ZENOH_PORT` (default `7447`) selects the peer UDP
scouting port as well as the existing coordinator TCP port. Set it on every
participating process to coexist with another Zenoh network:

```bash
export MINOT_ZENOH_PORT=17447
```

The peer protocol is isolated by `MINOT_DOMAIN_ID`, message type, and a versioned
key namespace. `local_only(true)` uses loopback listeners and loopback scouting.
Peers never probe or bind the coordinator TCP endpoint.

For Docker networks or routed meshes without multicast, set `MINOT_PEERS` to a
comma-separated list of reachable bootstrap endpoints, or configure them in
Rust. These are ordinary peers, and gossip discovers additional connections:

```rust
// A bootstrap peer: expose a reachable TCP port in the container network.
let seed = Node::create(NodeConfig::new("seed")
    .listen_endpoint("tcp/0.0.0.0:17447")
    .peer_endpoints(vec![])).await?;

// Another peer: no Minot coordinator required.
let node = Node::create(NodeConfig::new("worker")
    .peer_endpoints(vec!["tcp/seed:17447".into()])).await?;
```

Explicit bootstrap endpoints disable multicast but keep gossip enabled.
Choose reachable advertised addresses and expose ports as required by the
container network; gossip cannot overcome an unreachable endpoint.
`MINOT_COORD_ADDR` remains for coordinated clients and `mt_rat`, while
`MINOT_PEERS` configures peer pub/sub.

Discovery is asynchronous. If the first sample must have a recipient, call
`publisher.wait_for_subscribers(Duration::from_secs(5)).await?` before publishing.
This waits for at least one subscription. For a known startup group, use
`publisher.wait_for_subscriber_count(8, Duration::from_secs(5)).await?` instead.
`subscriber_count()` reports the currently discovered receiving endpoints.
Startup QoS checks and counts retain partial discovery results when a growing
mesh reaches its query timeout; later discovery still rejects incompatible
writers. Reliable sends require a complete subscriber snapshot and all queue
acknowledgements. Local-only peers elect one connection initiator per pair to
avoid redundant connection attempts during simultaneous startup.

`Reliable` delivery collects acknowledgements after samples enter subscriber
queues, and a missing acknowledgement cancels the peer run. `TryReliable`
uses reliable transport with a bounded send budget, without application ACKs;
`BestEffort` drops samples when its queue is full. A reliable subscriber rejects
a best-effort writer; incompatibility discovered later closes that subscription.
Transient-local publishers serve their latest sample to newly discovered
subscribers; sequence numbers remove overlap between snapshots and live data.
Only retained writers advertise a cache, so discovering volatile writers does
not trigger retained-sample queries.

The workspace currently pins Zenoh to merged upstream fix
[`646f2d1b` / #2779](https://github.com/eclipse-zenoh/zenoh/pull/2779), which removes
a startup lock inversion under peer churn. Zenoh 1.10.1 does not contain that
fix; the pin can return to a registry version once a release includes it.
Published Minot packages must wait for that fixed Zenoh release rather than
falling back to the affected registry version.
Fatal reliable-node loss is detected through Zenoh liveliness. Peer mode uses
`Timing::registration_timeout_ms` for ACK/send budgets; coordinator heartbeat
and reconnect policies apply only to coordinated modes. `connection()` reports
local session lifetime in peer mode, without coordinator generations.

The peer path currently uses owned Zenoh payload buffers. Minot's explicit SHM
pool controls and transport counters remain specific to coordinated transport.

For the legacy coordinator, `MINOT_ZENOH_PORT` controls its listener and local
client/probe endpoints. Explicit `MINOT_COORD_ADDR` keeps its specified port.
Invalid or zero port values warn and fall back to `7447`. Embedded coordinator
election remains scoped to the current user and port; separate Docker
filesystems do not share that lock. Use a designated coordinator and
`CoordMode::External` for coordinated applications across containers.

## Delivery modes

`Qos` combines `Reliability` and `Durability`. The table below describes the
coordinated backend; peer delivery behavior is described above.

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
