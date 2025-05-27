Minot is the name of the TUI you usually run but also of the project with all of its modules and other binaries. They can run distributed over the network or be embedded into a single process in the TUI itself.

There is no *typical* use case for Minot so you are expected to add flags to match your needs and build from source.

??? "Publishing custom and any-type messages"

    Publishing any-type messages from a bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.

**Building from source is easy.**

First you need to have a recent version of the [Rust Toolchain](https://www.rust-lang.org/tools/install) installed on your system. Then you can select your features for embedding more modules into the TUI.

A detailed list of supported embeddings can be found in the [Minot features definition](https://github.com/uos/minot/blob/main/minot/Cargo.toml#L45). Flags are additive, so every combination of flags works. For the TUI, they all just add more modules directly into one binary.

In default settings, Minot builds with an integrated coordinator. When running the following command with no changes, you will get a `minot` binary with integrated coordinator and a `minot-coord` binary, that is *just* a coordinator. For just sharing variables with the rat library TODOLINK, this is enough.


## Minot TUI

This version does not need any ROS1 or ROS2 installation to compile but it expects to find a node in the network if you try to query a bagfile.

~~~bash
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked
~~~

Here are some other typical combinations.

~~~bash title="ROS2 publisher (with any-type, needs sourced ROS2)" hl_lines="4"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ros2-c
~~~


~~~bash title="Ratpub publisher (no any-type)" hl_lines="4"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ratpub
~~~


~~~bash title="ROS1 and ROS2 publisher in the same process with no ROS installation (no any-type)" hl_lines="4"
cargo install \
  --git ssh://git@github.com/uos/minot.git minot \
  --locked \
  --features embed-ros-native
~~~


## Wind turbines (bagfile publishing nodes)

You can compile the embedded publishers standalone and distribute them in your network. For example: you could run the TUI on a Mac, connected to the Robot over WIFI or LAN, which runs ROS1 or ROS2 and the wind nodes.


They are inside the `wind` module. You can compile/install them to your `$PATH` by changing the previous command from `minot` to `wind`.

With `--all-features`, you get all binaries but you need a to have ROS2 sourced.

~~~bash title="ROS1 and ROS2 publisher in the same process with no ROS installation (no any-type)" hl_lines="4"
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

## Rats (sharing variables)

dd


## Uninstall

With modern software the word *"installing"* is overloaded. Cargo just compiles the code and moves the finished binaries to `~/.cargo/bin`. There are no other side effects or files from Minot.

If you want to remove them with cargo, you can `cargo uninstall minot` or `cargo uninstall wind` and they are gone.
