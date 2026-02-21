# Ratpub

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add these lines to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
[dependencies]
ratpub = "0.5.3"
tokio = { version = "1.49", features = ["full"] }
~~~

Since you probably want to use existing ROS 2 message definitions, you can also add the following crate, which is auto-generated from the Jazzy release. It bundles all usual types and implements the required `rkyv` traits for sending them over the Minot network.

~~~toml title="Cargo.toml"
[dependencies]
ros2-interfaces-jazzy-rkyv = { version = "0.0.5", features = [
  "std_msgs", # add more here
] }
~~~

Learn more on how to use it in your Code by visiting the [feature page](../features/pubsub.md).



