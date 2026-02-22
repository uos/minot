# ROS 2

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

