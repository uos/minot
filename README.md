# Minot

Minot is a highly versatile toolset for debugging and verifying stateful robot perception software. Some common use cases are:

* Fine-grained rosbag publishing
* Synchronous, deterministic and reproducable testing
* ROS1 -> ROS2 or language migrations
* Functional method evaluations

Visit the [Documentation](https://uos.github.io/minot) to find out more.

## Quick Install

For most users, the easiest way to install Minot is using the installation script with a single command. This will install the `minot` binaries and libraries into your user directory.

~~~bash
curl -sSLf https://install.steado.tech/minot | sh
~~~

For ROS support, make sure to have your ROS environment sourced before running the script.

After your first steps with Minot, you may want to use its more advanced features. Check the detailed [install page](https://uos.github.io/minot/installation.html) for that.

### VS Code Extension

Search for "Minot" in your editor an install the package. Running it will require a Minot binary in your `$PATH`. The extension will add syntax highlighting for `.mt` files and automatically activates as soon as you open a Minot file. You will see some buttons in the editor footer. Maybe start by selecting some lines and run them with `Run Selection`. Minot will be run automatically in the background.

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
