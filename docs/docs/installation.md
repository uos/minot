## Quick Install

For most users, the easiest way to install Minot is using the installation script with a single command:

~~~bash title="Install latest version (one-line command)"
curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh
~~~

You can also pass arguments to customize the installation:

~~~bash title="Install with ROS2 Jazzy support"
curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh -s -- --ros-distro jazzy
~~~

~~~bash title="Install specific version"
curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh -s -- --version v0.1.0-rc.5
~~~

~~~bash title="Build from source with embedded ROS2 native support"
curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh -s -- --build --embed ros2
~~~

~~~bash title="Build with multiple embedded components"
curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh -s -- --build --embed coord,ros1,ros2
~~~

Or download and run the script manually for more control:

~~~bash
wget https://raw.githubusercontent.com/uos/minot/main/install.sh
chmod +x install.sh
./install.sh --help
~~~

The installation script will:

- Automatically detect your operating system and architecture
- Download prebuilt binaries from GitHub releases if available
- Fall back to building from source with `cargo install` if needed
- Install binaries to `~/.local/bin` (customizable with `--dir`)
- Offer to install Rust automatically if needed for building from source

**Embedded Components:**

Use the `--embed` option to specify which components to embed when building from source:

- `coord` - Embedded coordinator (default)
- `ros1` - ROS1 publisher (native, no system dependencies)
- `ros2` - ROS2 publisher (native, no system dependencies)
- `ros2-c` - ROS2 publisher (C API, needs sourced ROS2)
- `ratpub` - Ratpub publisher
- `ros` - Both ROS1 and ROS2-C (needs both sourced)
- `ros-native` - Both ROS1 and ROS2 native (no system dependencies)

**Common usage examples:**

~~~bash
# Install with ROS2 Jazzy support
./install.sh --ros-distro jazzy

# Install with ROS2 Humble support
./install.sh --ros-distro humble

# Install specific version
./install.sh --version v0.1.0-rc.5

# Build from source with embedded components
./install.sh --build --embed ros2

# Build with multiple components
./install.sh --build --embed coord,ros1,ros2

# Install to custom directory
./install.sh --dir /usr/local/bin

# Non-interactive mode (useful for CI/CD)
./install.sh --yes --build --embed ros-native
~~~

## Prebuilt Binaries

Binaries for common system configurations are available [here](https://github.com/uos/minot/releases).

The `minot-$ROS_DISTRO-*` archives will only give you ROS specific binaries:

- `minot` TUI + Coordinator + ROS2 Publisher (with any-type)
- `wind-ros2-c` Standalone ROS2 Publisher (with any-type)

Archives not specific to a ROS distribution contain builds that can be used independently.

- `minot` TUI + Coordinator
- `minot-coord` Standalone Coordinator
- `librat.*, rat.h` C libraries for Variable Sharing
- `wind-ros*-native` ROS Publisher using [roslibrust](https://crates.io/crates/roslibrust) for ROS1 and [ros2-client](https://crates.io/crates/ros2-client) for ROS2

While the prebuilt binaries cover many common use cases, you may need to build Minot from source to tailor it to your specific needs. Building from source requires the [Rust toolchain](https://www.rust-lang.org/tools/install) to be installed on your system.

## Minot TUI

To build the Minot TUI, there are some feature flags for embedding common network participants for convenience. A detailed list of supported flags can be found in the [Minot features definition](https://github.com/uos/minot/blob/main/minot/Cargo.toml#L45).

With default settings, Minot builds with an integrated coordinator. When running the following command with no changes, you will get a `minot` binary with integrated Coordinator and a `minot-coord` binary, that is *just* a Coordinator. The standalone Coordinator is enough for sharing variables [with the rat library](./varshare.md#library).


??? "Publishing custom and any-type messages"

    Publishing any-type messages from a Bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.


**To help you get up and running quickly, here are some popular configurations.**

~~~bash title="(Recommended) With ROS2 publisher (+ any-type, needs sourced ROS2)"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked \
  --features embed-ros2-c
~~~

---

The next version does not need any ROS1 or ROS2 installation to compile but it expects to find a node in the network if you try to query a Bagfile:

~~~bash title="Minimal with embedded Coordinator"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked
~~~

Or maybe you want to publish to ROS1 and ROS2 at the same time without needing a ROS installation. Since the input is always a ROS2 Bagfile, only mapped types can be published (no any-type).

~~~bash title="With ROS1 and ROS2 publishers"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked \
  --features embed-ros-native
~~~

## Standalone Coordinator

Installing Minot TUI as shown above will also build a standalone variant of the Coordinator. It is called `minot-coord` in your path.

## Standalone Wind Turbines (Bagfile Publishers)

!!! tip

    You can compile the embedded publishers standalone and distribute them in your network.

    For example: you could run the TUI on a Mac, connected to a Robot with a Raspberry Pi over Wi-Fi or LAN, which runs ROS1 or ROS2 and the wind nodes.


Standalone publishers live inside the `wind` module. You can compile/install them to your `$PATH` by changing the previous command from `minot` to `wind`.

With `--all-features`, you'll get all binaries but you need to have ROS2 sourced.

~~~bash title="Standalone Publishers (needs sourced ROS2)"
cargo install \
  --git https://github.com/uos/minot wind \
  --locked \
  --all-features
~~~

The following flavours are available for publishing bagfile messages when specifying the respective feature:

- wind-ros2-c (with any-type) `--feature ros2-c`
- wind-ros1-native `--feature ros1-native`
- wind-ros2-native `--feature ros2-native`
- wind-rat `--feature ratpub`

Only the C version requires a ROS2 installation at compile time.

## Rats (Variable Sharing)

Nodes in the Minot network are called Rats ([here is why](./lore.md)). The functionality is shipped as a Rust and C library.

<!-- You can get the precompiled shared or static library including the header file here. -->

You need to build the library from source. It generates a static and shared library in the `./target/release/` folder. You will need to clone the repository first.

~~~bash title="Build librat from source"
git clone https://github.com/uos/minot
cd minot
cargo build --package rat --release
~~~

A typical system-wide installation is to copy the libraries to your linker path. Alternatively, you may change the link path and include search paths in your build system.

~~~bash
sudo cp ./target/release/librat.* /usr/local/lib/
sudo mkdir -p /usr/local/include/rat/
sudo cp ./rat/rat.h /usr/local/include/rat/
~~~

Then you can use the library in your C/C++ code.
~~~C
#include <rat/rat.h>
~~~

And link with `-lrat`.

---

For using the Rust library, just add this to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
rat = { version = "0.1.0-rc.5", git = "https://github.com/uos/minot" }
~~~

## Ratpub (Native Publish/Subscribe)

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add these lines to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
ratpub = { version = "0.1.0-rc.5", git = "https://github.com/uos/minot" }
tokio = { version = "1", features = ["full"] }
~~~

Since you probably want to use existing ROS2 message definitions, you can also add the following crate which is auto-generated from the Jazzy release. It bundles all usual types and implements the required `rkyv` traits for sending them over the Minot network.

~~~toml title="Cargo.toml"
ros2-interfaces-jazzy-rkyv = { version = "0.0.4", features = [
  "std_msgs", # add more here
], git = "https://github.com/stelzo/ros2-interfaces-jazzy-rkyv.git" }
~~~

Learn more on how to use it in your Code by visiting the [feature page](./pubsub.md).


## Uninstall

With modern software the word *"installing"* is overloaded. Cargo just compiles the code and moves the finished binaries to `~/.cargo/bin`. There are no other side effects or files from Minot.

If you want to remove them with cargo, you can `cargo uninstall minot` or `cargo uninstall wind` and they are gone.
