# Introduction

**Minot** [mai·not] is a robot development toolset written in Rust and developed at the Computer Engineering Group at the Osnabrück University. It provides tools for developing and verifying robot perception software.

The main feature is the ability to see ROS 2 MCAP Bagfiles as queryable timeseries data. Minot uses its [own simple language](features/bagquery.md) for configuration and querying.

~~~bash
minot tui <yourquery.mt>
~~~

Minot also runs as a ROS node. In that case, just prefix the usual ROS CLI:

~~~bash
ros2 run minot minot tui <yourquery.mt>
~~~

Refer to the [installation](installation/ros.md) for getting your Minot binaries.

All the provided features are meant to be used at development time to give you full control over the incoming data. It gives you transparency for complex systems where thousands of small sensor data packets are processed each second. With Minot, you regain control and explainability over what's happening when developing and integrating your Robot.

[Try it for yourself!](intro/query.md)

---

**Independence**

Minot is a collection of binaries and libraries. They can be used in your existing ROS1 or ROS2 nodes or completely separate from any system. This allows cross-ROS communication using its own networking layer without adding dependencies. With Minot, you can also write, test and debug ROS perception nodes without needing ROS on your system (currently Rust-only). This is especially useful for testing, where determinism is essential. When you are done, just comment out a few lines or change a compile flag and you are ready to run your node in ROS.

---

Minot was developed out of frustration. Maybe you know the pain as well:

!!! question "The Pain"

    === "Message Timing"

        Does your system behave different each time you play a Bagfile until you find out it actually works when playing it a little bit slower?

    === "Ignore Networking"

        Have you ever asked yourself why you should care about QoS settings, out-of-order message pipelines or the dependency of ROS2 itself when developing a Proof of Concept?

    === "Reproducability"

        Have you ever developed a ROS Node with state that unexpectedly changes for some reason and now you try to reproduce it?

