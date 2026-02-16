Minot consists of libraries and binaries, some with different compile flags for many use cases.

## ROS 2

Minot seamlessly works with ROS to integrate into existing robot pipelines.

### Binaries

We precompile the CLI with coordinator and ROS 2 publisher for our PPA. After the [setup](https://codeberg.org/uos-robotics/ppa/src/branch/pages/README.md), you can simply run apt.

~~~bash
# humble
sudo apt install ros-humble-minot

# jazzy
sudo apt install ros-jazzy-minot
~~~

### Source

On Jazzy and Humble, you'll need to [install a more modern Rust compiler](https://www.rust-lang.org/tools/install) first. The recommended script will automatically give you a newer version than 1.85, which is all we need.

The Minot CLI integrates seamlessly with typical ROS tooling. Just clone the repository into the src folder of your ROS workspace.

~~~bash
cd ~/ros2_ws/src
git clone https://github.com/uos/minot
cd ..

rosdep install --from-paths src -y --ignore-src

colcon build --packages-select minot
source install/local_setup.bash
~~~

Building will take a while.

Now run it like any ROS node.

~~~bash
ros2 run minot minot tui <file.mt>
~~~

!!! warning "High Disk Space Requirement"

    Building from source will create huge incremental cache artifacts. We recommend to use the binary installation wherever possible.

## Cargo

Minot is on [crates.io](https://crates.io/crates/minot), so Rustaceans can just use Cargo as expected.

This next line assumes ROS2 is installed and sourced.

~~~bash
cargo install minot --locked --features embed-ros2-c
~~~

The binary helper `cargo-binstall` is not supported because it does not support feature flags.

## Script

For most users, the easiest way to install everything Minot offers is using the installation script with a single command. This will install the `minot` binaries and libraries into your user directory.

~~~bash
curl -sSLf https://uos.github.io/minot/install | sh
~~~

For ROS support, make sure to have your ROS environment sourced before running the script.

You can now run Minot.

~~~bash
minot --help
~~~

Or the standalone coordinator.

~~~bash
minot-coord --help
~~~

If the command could not be found, add your local binary folder to your `$PATH`: `echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc`.

**Script Usage:**

Use the `--ros-distro` option to specify which ROS2 publisher bindings to embed when building from source:

- `jazzy` - ROS2 publisher (C API, needs sourced ROS2)
- `humble` - ROS2 publisher (C API, needs sourced ROS2)

Use the `--embed` option to specify which components to embed when building from source:

- `coord` - Coordinator (default)
- `ratpub` - Ratpub publisher
- `ros1-native` - ROS1 publisher (native, no system dependencies)
- `ros2-native` - ROS2 publisher with RustDDS (native, no system dependencies)

~~~bash title="Script arguments"
curl -sSLf https://uos.github.io/minot/install | sh -s -- --help
~~~

## VS Code Extension

Search for "Minot" in your editor an install the package. Running it will require a Minot binary in your `$PATH`. The extension will add syntax highlighting for `.mt` files and automatically activates as soon as you open a Minot file. You will see some buttons in the editor footer.

Select some lines and run them with `Run Selection` or use the Command Palette <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> and type "minot" to also see their keybindings.

## Tree-sitter Syntax Highlighting

Minot comes with Tree-sitter syntax for highlighting outside of VS Code. See [this repository](https://codeberg.org/stelzo/tree-sitter-minot) for instructions on how to add Minot support to the Helix editor or use the repository for other editors that support Tree-sitter grammars.

## Prebuilt Binaries

Binaries for common system configurations are available [here](https://github.com/uos/minot/releases).

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

---

## Embedded Ratpub Native Pub/Sub

For non-ROS use cases like standalone pipelines or CI/CD, we also offer packages with embedded coordinator and Ratpub publisher.

!!! info "Ratpub coordinator"
    
    Every install of Minot also gives you a standalone coordinator `minot-coord`. It is also a ready-to-use Ratpub coordinator for your pub/sub applications, which just use Ratpub and nothing else from Minot.

### Arch

Minot is in the AUR. Just use your favourite AUR helper.

~~~bash
paru -S minot
~~~

### Ubuntu

After setting up [our PPA](https://codeberg.org/uos-robotics/ppa/src/branch/pages/README.md):

~~~bash
sudo apt install minot
~~~

### PyPI

~~~sh
pip install minot-cli
~~~

---

## Bagfile Publishers

!!! tip

    You can compile the embedded publishers standalone and distribute them in your network.

    For example: you could run the TUI on a Mac, connected to a Robot with a Raspberry Pi over Wi-Fi or LAN, which runs ROS1 or ROS2 and the wind nodes.


Standalone publishers live inside the `wind` module. With `--all-features`, you'll get all binaries but you need to have ROS2 sourced.

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

### Ubuntu

Our PPA provides `.deb` files for a system-wide installation. After the [setup](https://codeberg.org/uos-robotics/ppa/src/branch/pages/README.md), you can simply run apt.

~~~bash
sudo apt install librat-dev
~~~

### Debian-based Distros

The PPA mentioned above is specific to Ubuntu the package itself does not require any system dependencies. Therefore it can be installed manually on all debian-based distros.

~~~bash title="Manual .deb Installation"
curl -s https://api.github.com/repos/uos/minot/releases/latest \
| grep "browser_download_url" \
| grep ".deb" \
| grep "$(dpkg --print-architecture)" \
| cut -d '"' -f 4 \
| xargs curl -L -O

sudo dpkg -i ./librat-dev_*.deb
~~~

The package also installs a pkg-config file, which allows the following usage in CMake.

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

### From Source

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
mt_rat = "0.5.2"
~~~

## Ratpub

Ratpub is only available for Rust. It uses Tokio for async I/O.
For using the library in your project, add these lines to your dependencies in `Cargo.toml`.

~~~toml title="Cargo.toml"
[dependencies]
ratpub = "0.5.2"
tokio = { version = "1.49", features = ["full"] }
~~~

Since you probably want to use existing ROS 2 message definitions, you can also add the following crate, which is auto-generated from the Jazzy release. It bundles all usual types and implements the required `rkyv` traits for sending them over the Minot network.

~~~toml title="Cargo.toml"
[dependencies]
ros2-interfaces-jazzy-rkyv = { version = "0.0.5", features = [
  "std_msgs", # add more here
] }
~~~

Learn more on how to use it in your Code by visiting the [feature page](./pubsub.md).


## Uninstall

If you want to remove every file installed from the install script, you can run `minot-uninstall`. This executes a generated shell script that saved all added files and now removes them (including itself). You can also run `minot uninstall` to uninstall minot and minot-coord. Or `cargo uninstall minot` if you installed binaries with cargo. The debian packages can be uninstalled with `sudo apt remove librat-dev ros-jazzy-minot` etc.
