# Bagfile Publishers

Standalone publishers live inside the `wind` module. With `--all-features`, you'll get all binaries but you need to have ROS2 sourced.

!!! tip

    You can compile the embedded publishers standalone and distribute them in your network.

    **For example**: run the sync command on a Mac, connected to a robot with a Raspberry Pi over Wi-Fi or LAN, which runs ROS1 or ROS2 and the wind nodes.


~~~bash title="Standalone Publishers (needs sourced ROS2)"
cargo install mt_wind --locked --all-features
~~~

The following flavours are available for publishing bagfile messages when specifying the respective feature:

- wind-ros2-c (with any-type) `--feature ros2-c`
- wind-ros1-native `--feature ros1-native`
- wind-ros2-native `--feature ros2-native`
- wind-mt-pubsub `--feature mt_pubsub`

Only the C version requires a ROS2 installation at compile time.

