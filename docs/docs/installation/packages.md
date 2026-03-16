# Packages

We package some variations of Minot for typical use cases outside the typical ROS toolchain.

## Cargo

Minot is on [crates.io](https://crates.io/crates/minot), so Rustaceans can just use Cargo as expected.

Since this method builds Minot from source, we can fully leverage the `--features` flags for selecting our embedded components:

- `embed-coord` - Coordinator (default)
- `embed-ros2-c` - ROS 2 publisher - needs sourced ROS environment
- `embed-ros2-c-humble` - ROS 2 Humble publisher - needs sourced ROS environment. *Required for Humble because of changes in QoS.*
- `embed-mt-pubsub` - MtPubSub publisher
- `embed-ros1-native` - ROS1 publisher (native, no system dependencies)
- `embed-ros2-native` - ROS2 publisher with RustDDS (native, no system dependencies)

~~~bash
cargo install minot --locked
~~~

The binary helper `cargo-binstall` is not supported because it does not support feature flags.

## Non-ROS

For standalone scripts, MtPubSub-native applications or CI/CD, we also offer pre-compiled packages with **embedded coordinator**, **MtPubSub publisher** and shell completions.

!!! info "MtPubSub coordinator"
    
    Every install of Minot already ships a standalone coordinator `minot-coord`. It is also a ready-to-use MtPubSub coordinator for your MtPubSub pub/sub applications.

### Arch

Minot is in the AUR. Just use your favourite AUR helper.

~~~bash
paru -S minot
~~~

### Ubuntu

Make sure you have our PPA.

~~~bash title="UOS PPA"
curl -fsSL "https://uos-robotics.codeberg.page/ppa/ubuntu/key.gpg" | gpg --dearmor \
  | sudo tee /usr/share/keyrings/uos-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/uos-archive-keyring.gpg] https://uos-robotics.codeberg.page/ppa/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/uos.list
sudo apt update
~~~

After the setup, you can simply run apt.

~~~bash
sudo apt install minot
~~~

### PyPI

~~~sh
pip install minot-cli
~~~

### Homebrew (MacOS)

There is a tap with precompiled binaries for Apple Silicon Macs.

~~~sh
brew tap uos/minot

brew install minot
~~~

---

