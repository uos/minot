Minot consists libraries and binaries, some with different compile flags for many use cases. A lot of ways to install various part of Minot are described on this page.

## Install Script (Recommended)

For most users, the easiest way to install everything Minot offers is using the installation script with a single command. This will install the `minot` binaries and libraries into your user directory.

~~~bash
curl -sSLf https://uos.github.io/minot/install | sh
~~~

For ROS support, make sure to have your ROS environment sourced before running the script.

## Using Cargo

Minot is on crates.io, so Rustaceans can just use Cargo as expected. This includes the `ratpub` crate as a library, but you will need a `minot-coord` running on the network for the ratpub nodes to communicate. The `minot-coord` binary is also installed with the following command.

~~~bash
cargo install minot
~~~


## VS Code Extension

Search for "Minot" in your editor an install the package. Running it will require a Minot binary in your `$PATH`. The extension will add syntax highlighting for `.mt` files and automatically activates as soon as you open a Minot file. You will see some buttons in the editor footer. Select some lines and run them with `Run Selection`. Hover over the buttons to see their keybindings.

## Tree-sitter Support

Minot comes with support for Tree-sitter syntax highlighting outside of VS Code. See [this repository](https://github.com/stelzo/tree-sitter-minot) for instructions on how to add Minot support to the Helix editor or use the repository for other editors that support Tree-sitter grammars.

## Advanced

You can also download and run the install script manually for more control.
Use `--help` to see all available configurations of Minot.

~~~bash
wget https://uos.github.io/minot/install -o install.sh
chmod +x install.sh
./install.sh --help
~~~

The installation script will:

- Automatically detect your operating system and architecture
- Resolve to the newest compatible and stable version
- Download prebuilt binaries from GitHub releases if available
- Fall back to building from source using Rust if needed
- Copy binaries and libraries into your local user directory (customizable with `--dir`)
- Add the uninstall script (`minot-uninstall`) to the same directory for easy removal later

**Embedded Components:**

Use the `--ros-distro` option to specify which ROS2 publisher bindings to embed when building from source:

- `jazzy` - ROS2 publisher (C API, needs sourced ROS2)
- `humble` - ROS2 publisher (C API, needs sourced ROS2)

Use the `--embed` option to specify which components to embed when building from source:

- `coord` - Embedded coordinator (default)
- `ratpub` - Ratpub publisher
- `ros1-native` - ROS1 publisher (native, no system dependencies)
- `ros2-native` - ROS2 publisher with RustDDS (native, no system dependencies)
- `ros2-c` - Use the sourced rclc and build bindings around it. Needs to have ROS2 installed and sourced
- `ros2-c-humble` - Use the sourced Humble rclc and build bindings around it. Needs to have ROS2 installed and sourced. Handles Humble specific QoS.

**Common usage examples:**

~~~bash
# Install with ROS2 Jazzy support
./install.sh --ros-distro jazzy

# Build with multiple components
./install.sh --build --embed ros1 --ros-distro jazzy

# Non-interactive mode (useful for CI/CD)
./install.sh --yes --build --embed ratpub
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

## All-in-one Binary

To build the Minot bundled binary, there are some feature flags for embedding common network participants for convenience. A detailed list of supported flags can be found in the [Minot features definition](https://github.com/uos/minot/blob/main/minot/Cargo.toml#L45).

With default settings, Minot builds with an integrated coordinator. When running the following command with no changes, you will get a `minot` binary with integrated Coordinator and a `minot-coord` binary, that is *just* a Coordinator. The standalone Coordinator is enough for sharing variables [with the rat library](./varshare.md#library).


??? "Publishing custom and any-type messages"

    Publishing any-type messages from a Bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.


**To help you get up and running quickly, here are some popular configurations.**

~~~bash title="(Recommended) With ROS2 publisher (+ any-type, needs sourced ROS2)"
cargo install minot --locked --features embed-ros2-c
~~~

---

This next version does not need any ROS1 or ROS2 installation to compile. It tries to find other standalone Minot nodes in the network and publishes to them if you query a Bagfile:

~~~bash title="Minimal with embedded Coordinator"
cargo install minot --locked
~~~

Or maybe you want to publish to ROS1 and ROS2 at the same time without needing a ROS installation. Since the input is always a ROS2 Bagfile, only mapped types can be published (no any-type).

~~~bash title="With ROS1 and ROS2 publishers"
cargo install minot --locked --features embed-ros1-native,embed-ros2-native
~~~

## Coordinator

Running `cargo install` as shown above will also build a standalone variant of the Coordinator. It is called `minot-coord` in your path.

## Bagfile Publishers

!!! tip

    You can compile the embedded publishers standalone and distribute them in your network.

    For example: you could run the TUI on a Mac, connected to a Robot with a Raspberry Pi over Wi-Fi or LAN, which runs ROS1 or ROS2 and the wind nodes.


Standalone publishers live inside the `wind` module. You can compile/install them to your `$PATH` by changing the previous command from `minot` to `wind`.

With `--all-features`, you'll get all binaries but you need to have ROS2 sourced.

~~~bash title="Standalone Publishers (needs sourced ROS2)"
cargo install mt_wind --locked --all-features
~~~

The following flavours are available for publishing bagfile messages when specifying the respective feature:

- wind-ros2-c (with any-type) `--feature ros2-c`
- wind-ros1-native `--feature ros1-native`
- wind-ros2-native `--feature ros2-native`
- wind-rat `--feature ratpub`

Only the C version requires a ROS2 installation at compile time.

## Rats

Nodes in the Minot network that share data are called Rats ([here is why](./lore.md)). The functionality is shipped as a Rust and C library.

### Debian-based Linux

There are prebuilt debian packages for the `rat` library.

~~~bash title="Download the .deb"
curl -s https://api.github.com/repos/uos/minot/releases/latest \
| grep "browser_download_url" \
| grep ".deb" \
| grep "$(dpkg --print-architecture)" \
| cut -d '"' -f 4 \
| xargs curl -L -O

sudo dpkg -i ./librat-dev_*.deb
~~~

The package also installed a pkg-config file, which allows the following usage in CMake.

~~~cmake title="Example CMake"
find_package(PkgConfig REQUIRED)
pkg_check_modules(RAT REQUIRED librat)

add_executable(my_app main.c)
target_include_directories(my_app PRIVATE ${RAT_INCLUDE_DIRS})
target_link_libraries(myfind_package(PkgConfig REQUIRED)
pkg_check_modules(RAT REQUIRED librat)

add_executable(my_app main.c)
target_include_directories(my_app PRIVATE ${RAT_INCLUDE_DIRS})
target_link_libraries(my_app PRIVATE ${RAT_LIBRARIES})
~~~

### From source

Building from source generates a static and shared library in the `./target/release/` folder. You will need to clone the repository first.

~~~bash title="Build librat from source"
git clone https://github.com/uos/minot
cd minot
cargo build --package mt_rat --release
~~~

A typical system-wide installation is done by copying the libraries to your linker path. Alternatively, you may change the link path and include search paths in your build system.

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
[dependencies]
mt_rat = "0.3.0"
~~~

## Ratpub

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add these lines to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
[dependencies]
ratpub = "0.3.0"
tokio = { version = "1", features = ["full"] }
~~~

Since you probably want to use existing ROS2 message definitions, you can also add the following crate which is auto-generated from the Jazzy release. It bundles all usual types and implements the required `rkyv` traits for sending them over the Minot network.

~~~toml title="Cargo.toml"
[dependencies]
ros2-interfaces-jazzy-rkyv = { version = "0.0.5", features = [
  "std_msgs", # add more here
] }
~~~

Learn more on how to use it in your Code by visiting the [feature page](./pubsub.md).


## Uninstall

If you want to remove every file installed from the install script, you can run `minot-uninstall`. This executes a generated shell script that saved all added files and now removes them (including itself). You can also run `minot uninstall` to uninstall minot and minot-coord. Or `cargo uninstall wind` if you installed binaries with cargo. The debian packages can be uninstalled with `sudo apt remove librat-dev`.
