# ROS 2

Minot seamlessly works with ROS to integrate into existing robot pipelines.

### Binaries

We precompile the CLI with coordinator and ROS 2 publisher for our PPA. 

~~~bash title="UOS PPA"
curl -fsSL "https://uos-robotics.codeberg.page/ppa/ubuntu/key.gpg" | gpg --dearmor \
  | sudo tee /usr/share/keyrings/uos-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/uos-archive-keyring.gpg] https://uos-robotics.codeberg.page/ppa/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/uos.list
sudo apt update
~~~

After the setup, you can install the matching package for any of the supported
ROS 2 releases. Lyrical, Jazzy, and Humble packages are built for amd64 and
arm64.

~~~bash
# humble
sudo apt install ros-humble-minot

# jazzy
sudo apt install ros-jazzy-minot

# lyrical
sudo apt install ros-lyrical-minot
~~~

You can also use the installation script after sourcing ROS. It selects the
release matching `$ROS_DISTRO` when a prebuilt archive is available and falls
back to a source build otherwise.

~~~bash
curl -sSLf https://stelzo.codeberg.page/minot/install | sh -s -- --ros-distro "$ROS_DISTRO"
~~~

### Source

On Lyrical, Jazzy, and Humble, you'll need to [install a more modern Rust compiler](https://www.rust-lang.org/tools/install) first. The recommended script will automatically give you a newer version than 1.85, which is all we need.

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
ros2 run minot minot sync <file.mt>
~~~

!!! warning "High Disk Space Requirement"

    Building from source will create huge incremental cache artifacts. We recommend to use the binary installation wherever possible.
