!!! warning

    There are no prebuilt binaries or packages yet.

    But you can easily install everything from source, it just takes a little bit longer. You'll need to have [Rust installed](https://www.rust-lang.org/tools/install).

Minot is the name of the TUI you usually run but also of the project with all of its modules and other binaries. They can run distributed over the network or be embedded into a single process in the TUI itself.

Because of this, there is no *typical* use case for Minot so you are expected to add flags to match your needs and build from source. But fear not, building from source is easy.


The only requirement is to have a recent version of the [Rust Toolchain](https://www.rust-lang.org/tools/install) installed on your system.

## Minot TUI

To build the Minot TUI, there are some feature flags for embedding common network participants for convenience. A detailed list of supported flags can be found in the [Minot features definition](https://github.com/uos/minot/blob/main/minot/Cargo.toml#L45).

With default settings, Minot builds with an integrated coordinator. When running the following command with no changes, you will get a `minot` binary with integrated Coordinator and a `minot-coord` binary, that is *just* a Coordinator. The standalone Coordinator is enough for sharing variables [with the rat library](./varshare.md#library).


??? "Publishing custom and any-type messages"

    Publishing any-type messages from a bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.


**Here are some common flavours of the Minot TUI to get you started quickly.**

~~~bash title="(Recommended) With ROS2 publisher (+ any-type, needs sourced ROS2)"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked \
  --features embed-ros2-c
~~~

---

The next version does not need any ROS1 or ROS2 installation to compile but it expects to find a node in the network if you try to query a bagfile:

~~~bash title="Minimal with embedded Coordinator"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked
~~~

Or maybe you want to publish to ROS1 and ROS2 at the same time without needing a ROS installation. Since the bagfile is ROS2, only mapped types are supported (no any-type).

~~~bash title="With ROS1 and ROS2 publishers"
cargo install \
  --git https://github.com/uos/minot minot \
  --locked \
  --features embed-ros-native
~~~

## Standalone Coordinator

Installing Minot TUI as shown above will also build a standalone variant of the Coordinator. It is called `minot-coord` in your path.

<!-- The Minot Coordinator can work as a standalone static binary. To make it easy to start, there is a binary distribution for common operating system and architectures on GitHub. -->


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
rat = { version = "0.1.0-rc.1", git = "https://github.com/uos/minot" }
~~~

## Ratpub (Native Publish/Subscribe)

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add these lines to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
ratpub = { version = "0.1.0-rc.1", git = "https://github.com/uos/minot" }
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
