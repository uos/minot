# Minot

Minot is a highly versatile toolset for debugging and verifying stateful robot perception software. Some common use cases are:

* Fine-grained rosbag publishing
* Synchronous, deterministic and reproducable testing
* ROS1 -> ROS2 or language migrations
* Functional method evaluations

Visit the [Documentation](https://uos.github.io/minot) to find out more.

## ROS2

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
ros2 run minot minot --help
~~~

Building from source will create huge incremental cache artifacts. To save time and space, we recommend the binary installation described in the [install documentation](https://uos.github.io/minot/installation.html).

### VS Code Extension

Search for "Minot" in the extension marketplace an install. The plugin requires a Minot binary in your `$PATH`. The extension will add syntax highlighting for `.mt` files and automatically activates as soon as you open a Minot file. You will see some buttons in the editor footer. Maybe start by selecting some lines and run them with `Run Selection`. Minot will be started automatically for you in the background.

More information about the extension can be found at the [Marketplace](https://marketplace.visualstudio.com/items?itemName=stelzo.minot).

### Tree-sitter Support

Minot comes with support for Tree-sitter syntax highlighting outside of VS Code. See [this repository](https://github.com/stelzo/tree-sitter-minot) for instructions on how to add Minot support to the Helix editor or use the repository for other editors that support Tree-sitter grammars.

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
