# ratpub

A simple and deterministic Pub/Sub implementation with Minot primitives.

Part of the [minot](https://github.com/uos/minot) family of crates.

[![Latest version](https://img.shields.io/crates/v/ratpub.svg)](https://crates.io/crates/ratpub)
![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Apache](https://img.shields.io/badge/license-Apache-blue.svg)


~~~toml title="Cargo.toml"
[dependencies]
ratpub = "0.4.4"
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

