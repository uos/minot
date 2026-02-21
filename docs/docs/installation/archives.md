# Raw Archives

Binaries for common system configurations are available [via GitHub Releases](https://github.com/uos/minot/releases).

The `minot-$ROS_DISTRO-*` archives will only give you ROS specific binaries:

- `minot` TUI + coordinator + ROS2 Publisher (with any-type)
- `wind-ros2-c` Standalone ROS2 Publisher (with any-type)

??? "Publishing custom and any-type messages"

    Publishing any-type messages from a Bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.

Archives not specific to a ROS distribution contain builds that can be used independently.

- `minot` TUI + coordinator
- `minot-coord` Standalone coordinator
- `librat.*, rat.h` C libraries for Variable Sharing
- `wind-ros*-native` ROS Publisher using [roslibrust](https://crates.io/crates/roslibrust) for ROS1 and [ros2-client](https://crates.io/crates/ros2-client) for ROS2

While the prebuilt binaries cover many common use cases, you may need to build Minot from source to tailor it to your specific needs. Building from source requires the [Rust toolchain](https://www.rust-lang.org/tools/install) to be installed on your system.

