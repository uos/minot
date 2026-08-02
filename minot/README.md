# Minot

## Local-only networking

Pass `--local-only` to `minot` or `minot-coord` to restrict Minot discovery and
communication to processes on the same machine:

```text
minot --local-only sync example.mt
minot-coord --local-only
```

This still uses loopback networking so local Minot processes can communicate.
If a Zenoh router is already listening on the local Minot endpoint
(`127.0.0.1:7447`), Minot warns and ignores `--local-only` so it can use the
existing router instead.
Without the option, Minot retains its default network-wide discovery behavior.

Minot is a highly versatile toolset for debugging and verifying stateful robot perception software. Some common use cases are:

* Fine-grained rosbag publishing
* Synchronous, deterministic and reproducable testing
* ROS1 -> ROS2 or language migrations
* Functional method evaluations

Visit the [Documentation](https://uos.github.io/minot) to find out more.

[![Latest version](https://img.shields.io/crates/v/minot.svg)](https://crates.io/crates/minot)
![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Apache](https://img.shields.io/badge/license-Apache-blue.svg)

~~~sh
cargo install minot
~~~
