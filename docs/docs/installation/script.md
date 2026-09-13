# Script

For most users, the easiest way to install everything Minot offers is using the installation script with a single command. This will install the `minot` binaries and libraries into your user directory.

~~~bash
curl -sSLf https://stelzo.codeberg.page/minot/install | sh
~~~

For ROS support, source your ROS environment and pass its distribution to the
script. Lyrical, Jazzy, and Humble are supported release targets.

~~~bash title="ROS 2 Lyrical"
source /opt/ros/lyrical/setup.bash
curl -sSLf https://stelzo.codeberg.page/minot/install | sh -s -- --ros-distro lyrical
~~~

You can now run Minot.

~~~bash
minot --help
~~~

Or the standalone coordinator.

~~~bash
minot coord --help
~~~

If the command could not be found, add your local binary folder to your `$PATH`: `echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc`.

**Script Usage:**

Use the `--ros-distro` option to select the matching prebuilt ROS 2 release or
the publisher bindings to embed when building from source:

- `lyrical` - ROS2 publisher (C API, needs sourced ROS2)
- `jazzy` - ROS2 publisher (C API, needs sourced ROS2)
- `humble` - ROS2 publisher (C API, needs sourced ROS2)

Use the `--embed` option to specify which components to embed when building from source:

- `coord` - Coordinator (default)
- `mt_pubsub` - MtPubSub publisher (default non-ROS backend)
- `ros1-native` - ROS1 publisher (native, no system dependencies)
- `ros2-native` - ROS2 publisher with RustDDS (native, no system dependencies)

~~~bash title="Script arguments"
curl -sSLf https://stelzo.codeberg.page/minot/install | sh -s -- --help
~~~
