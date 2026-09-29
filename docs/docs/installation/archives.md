# Release Binaries

Every [Codeberg Release](https://codeberg.org/stelzo/minot/releases) contains the following files, each with a matching `.sha256` next to it.

**Minot** as a single binary per target:

- `minot-<target>` sync + coordinator (`minot coord`) + MtPubSub publisher, for `x86_64-unknown-linux-gnu`, `x86_64-unknown-linux-musl`, `aarch64-unknown-linux-gnu`, `armv7-unknown-linux-gnueabihf` and `aarch64-apple-darwin`
- `minot-jazzy-<target>` and `minot-humble-<target>` sync + coordinator + ROS2 publisher (with any-type), for `x86_64-unknown-linux-gnu` and `aarch64-unknown-linux-gnu`
- `minot-humble-ros1-<target>` the same as `minot-humble-<target>` plus a ROS1 publisher

The ROS builds are tied to the named ROS 2 distribution. The installer checks that the matching ROS environment is available before using one.

!!! note "Publishing custom and any-type messages"

    Publishing any-type messages from a Bagfile (like `ros2 bag play`) needs the ROS2 C implementation to be linked to the message. When building the Rust bindings, it will link with every message in your `$PATH`. So if you use custom messages, you want to build Minot after you sourced your new message.

**The rat library** for [Variable Sharing](librat.md) in C and C++:

- `librat-<target>.a` static library, for every target above that is not ROS specific
- `librat-<target>.so` (Linux GNU) or `librat-<target>.dylib` (macOS) shared library
- `rat.h`, `librat.pc` and `libratConfig.cmake`, the same for every target

The [installation script](script.md) installs these files with `--with-rat`.

The standalone bagfile publishers are not part of the releases. [Build them from source](publisher.md) if you need them. For anything the prebuilt binaries do not cover, build Minot from source, which requires the [Rust toolchain](https://www.rust-lang.org/tools/install).
