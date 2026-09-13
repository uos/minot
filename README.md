# Minot

Minot is a highly versatile toolset for debugging and verifying stateful robot perception software. Some common use cases are:

* Fine-grained rosbag publishing
* Synchronous, deterministic and reproducable testing
* ROS1 -> ROS2 or language migrations
* Functional method evaluations

Visit the [Web Documentation](https://stelzo.codeberg.page/minot) or `ssh minot@steado.tech` to find out more.

## ROS 2

### Binary Release

We precompile the CLI with coordinator and ROS 2 publisher for our PPA. 

~~~bash title="UOS PPA"
curl -fsSL "https://uos-robotics.codeberg.page/ppa/ubuntu/key.gpg" | gpg --dearmor \
  | sudo tee /usr/share/keyrings/uos-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/uos-archive-keyring.gpg] https://uos-robotics.codeberg.page/ppa/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/uos.list
sudo apt update
~~~

After the setup, you can install the matching package for any of the supported ROS 2 releases. Lyrical, Jazzy, and Humble packages are built for amd64 and arm64.

~~~bash
# humble
sudo apt install ros-humble-minot

# jazzy
sudo apt install ros-jazzy-minot
~~~

Alternatively, with the matching ROS environment sourced, the installer can select the corresponding prebuilt release or build it from source:

~~~bash
curl -sSLf https://stelzo.codeberg.page/minot/install | sh -s -- --ros-distro "$ROS_DISTRO"
~~~

### From Source

On Jazzy, and Humble, you'll need to [install a more modern Rust compiler](https://www.rust-lang.org/tools/install) first. The recommended script will automatically give you a newer version than 1.85, which is all we need.

The Minot CLI integrates seamlessly with typical ROS tooling. Just clone the repository into the src folder of your ROS workspace.

~~~bash
cd ~/ros2_ws/src
git clone https://codeberg.org/stelzo/minot
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

Building from source will create huge incremental cache artifacts. To save time and space, we recommend the binary installation described in the [install documentation](https://stelzo.codeberg.page/minot/installation/packages.html).

### VS Code Extension

Install "Minot" from the extension marketplace and place the Minot binary in `$PATH`. Opening an `.mt` file enables syntax highlighting and editor actions. Select a few lines and choose `Run Selection` to start Minot in the background.

More information about the extension can be found at the [Marketplace](https://marketplace.visualstudio.com/items?itemName=stelzo.minot).

### Tree-sitter Support

Minot comes with support for Tree-sitter syntax highlighting outside of VS Code. See [this repository](https://codeberg.org/stelzo/tree-sitter-minot) for instructions on how to add Minot support to the Helix editor or use the repository for other editors that support Tree-sitter grammars.

### License

<sup>
Licensed under either of <a href="LICENSE-APACHE">Apache License, Version
2.0</a> or <a href="LICENSE-MIT">MIT license</a> at your option.
</sup>

<br>

<sub>
Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this crate by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
</sub>
