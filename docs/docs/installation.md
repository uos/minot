Minot is the name of the TUI you usually run but also of the project with all of its modules and other binaries. They can run distributed over the network or be embedded into a single process in the TUI itself.

Because of this, there is no *typical* use case for Minot so you are expected to add flags to match your needs and build from source. But fear not, building from source is easy.


The only requirement is to have a recent version of the [Rust Toolchain](https://www.rust-lang.org/tools/install) installed on your system.

## Minot TUI

To build the Minot TUI, there are some feature flags for embedding common network participants for convenience. A detailed list of supported flags can be found in the [Minot features definition](https://github.com/uos/minot/blob/main/minot/Cargo.toml#L45).

With default settings, Minot builds with an integrated coordinator. When running the following command with no changes, you will get a `minot` binary with integrated Coordinator and a `minot-coord` binary, that is *just* a Coordinator. The standalone Coordinator is enough for sharing variables [with the rat library](./varshare.md#library).


??? "Publishing custom and any-type messages"

    Publishing any-type messages from a bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.


**Here are some common flavours of the Minot TUI to get you started quickly.**

This version does not need any ROS1 or ROS2 installation to compile but it expects to find a node in the network if you try to query a bagfile.

~~~bash title="With ROS2 publisher (with any-type, needs sourced ROS2)"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ros2-c
~~~

---

~~~bash title="Minimal with embedded Coordinator - expects nodes in the network when publishing"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked
~~~



~~~bash title="With Ratpub publisher (no any-type)"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ratpub
~~~


~~~bash title="With ROS1 and ROS2 publisher in the same process with no ROS installation (no any-type)"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ros-native
~~~

## Standalone Coordinator

The Minot Coordinator can work as a standalone static binary. To make it easy to start, there is a binary distribution for common operating system and architectures on GitHub.


## Standalone Wind Turbines (Bagfile Publishers)

You can compile the embedded publishers standalone and distribute them in your network. For example: you could run the TUI on a Mac, connected to the Robot over WIFI or LAN, which runs ROS1 or ROS2 and the wind nodes.

### Prebuild Binaries
bla


### Build From Source

They are inside the `wind` module. You can compile/install them to your `$PATH` by changing the previous command from `minot` to `wind`.

With `--all-features`, you'll get all binaries but you need a to have ROS2 sourced.

~~~bash title="ROS1 and ROS2 publisher in the same process with no ROS installation (no any-type)"
cargo install \
  --git ssh://git@github.com/uos/minot.git wind \
  --locked \
  --all-features
~~~

The following binaries are available for publishing bagfile messages when specifying the respective feature:

- wind-ros1-native `--feature ros1-native`
- wind-ros2-native `--feature ros2-native`
- wind-ros2-c (with any-type) `--feature ros2-c`
- wind-rat `--feature ratpub`

Only the C version requires a ROS2 installation at compile time.

## Rats (Variable Sharing)

Nodes in the Minot system are called Rats ([here is why](./lore.md)). The functionality is shipped as a Rust and C library. You can get the precompiled shared or static library including the header file here.

You can also build the library from source. It generates a static and shared library in the `./target/release/` folder. You will need to clone the repository first.

~~~bash title="Build librat from source"
git clone https://github.com/uos/minot
cd minot
cargo build --package rat --release
~~~

A typical installation is to copy the libraries to your system directory. Alternatively, change the link and include search paths.

~~~bash
sudo cp ./target/release/librat.* /usr/local/lib/
sudo mkdir -p /usr/local/include/rat/
sudo cp ./rat/rat.h /usr/local/include/rat/
~~~

Then you can use them in your C/C++ code.
~~~C
#include <rat/rat.h>
~~~

And link with `-lrat`.

---

For using the Rust library, add this to your `Cargo.toml`.

~~~toml title="Cargo.toml"
rat = { version = "0.1.0-rc.1", git = "ssh://git@github.com/uos/minot.git" }
~~~

## Ratpub (Native Publish/Subscribe)

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add this line to your `Cargo.toml`.

~~~toml title="Cargo.toml"
ratpub = { version = "0.1.0-rc.1", git = "ssh://git@github.com/uos/minot.git" }
tokio = { version = "1", features = ["full"] }
~~~

Learn more on how to use it in your Code by visiting the [feature page](./pubsub.md).


## Uninstall

With modern software the word *"installing"* is overloaded. Cargo just compiles the code and moves the finished binaries to `~/.cargo/bin`. There are no other side effects or files from Minot.

If you want to remove them with cargo, you can `cargo uninstall minot` or `cargo uninstall wind` and they are gone.
