# Query a Bagfile

With ROS you can record and play everything published to the ROS network. This enables reproducability for nodes and greatly improves the safety of developing real robots. Bagfiles are great! But the ROS tooling for playing the Bagfile is limited. It's still enough for most use cases but there are times where you need complete control over the data that is published to your subscribers. Minot packages its own language for fine grained filtering and cursor control inside your Bagfile for fast iterations. This is called *Bagfile Querying* in Minot.

## Quickstart

To get started quickly, we will do a very short Minot *Hello World*.

We require a ROS 2 Jazzy or Humble installation for this demo because we want to see our point clouds in RViz.

~~~ bash title="ROS Example"
sudo apt install curl unzip

mkdir minot-bagfile-demo && cd minot-bagfile-demo
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

# Get Minot with ROS2 publisher
curl -sSLf https://uos.github.io/minot/install | sh -s -- --ros-distro $ROS_DISTRO

# Get a Bagfile saved with mcap storage
curl -Lo dlg_cut.zip "https://myshare.uni-osnabrueck.de/f/5faf4154af384854ab94?dl=1" \
    && unzip dlg_cut.zip \
    && rm dlg_cut.zip

# Get the query
curl -Lo demo.mt "https://uos.github.io/minot/assets/demo_publish.mt"

# Get the rviz demo preset
curl -Lo demo.rviz "https://uos.github.io/minot/assets/demo_publish.rviz"

# Run rviz and wait for data
rviz2 -d demo.rviz
~~~

Open up a new terminal.

~~~ bash title="Minot TUI"
source /opt/ros/$ROS_DISTRO/setup.bash

# Run
minot tui demo.mt

# Press Space
# You should see a 3D point cloud in rviz2 now.
# Press j until you see "W3" in the bottom right.
# Press Space again to evaluate line 3 of demo.mt and repeat how often you like.
# Quit Minot with q

# Clean up everything we created
cd .. && rm -rf minot-bagfile-demo && minot-uninstall
~~~

You just did a very basic **Bagfile Query**. It makes up one of 3 major features in Minot. They can be used standalone but when combined, they enable superpowers.

