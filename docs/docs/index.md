# Overview

Minot provides primitives for developing and validating stateful robot software.

Minot was the result out of the following painpoints. When developing, we want to solve a small problem that plays nicely with the larger system. Modularity is one of the main goals of ROS. And while developing and integrating these modules, Minot is here to help.

!!! question "Painpoints"

    === "Message Timing Issues"

        Does your system behave differently each time you play a Bagfile until you find out it actually works when playing the Bagfile a little bit slower?

    === "Ignore Networking"

        Have you ever asked yourself why you should care about QoS Settings, out-of-order message pipelines or the dependency of ROS2 itself when developing a Proof of Concept?

    === "Reproducability"

        Did you ever develop a ROS Node with state that unexpectedly changes for some reason and now you try to reproduce it?

Minot is meant to be used at development time to give you full control over the incoming data. It gives you transparency for complex systems where thousands of small sensor data packets are processed each second. With Minot, you regain control and explainability over what's happening when developing and integrating your Robot.

**How?**

Minot is a collection of binaries and libraries. Some of them usable in any existing ROS1 or ROS2 node, others completely separate from any system. This allows cross-ROS communication using its own networking layer without adding dependencies. With minot you can write, test and debug ROS perception nodes without needing ROS on your system.

This is enabled by 3 features, we call them *primitives*. They can be used standalone but when combined, they enable superpowers.

## Sneak Peak
<!-- === "Technical Debt" -->

<!-- Have you ever tried to develop a new algorithm for just a small subproblem of a larger system and now you need to use all those tools of the old method? -->


TODO VIDEO!

!!! info "What is going on?"

    You are looking at a Lidar Inertial Odometry ROS2 node written in Rust. For IMU initialisation, just the needed frames are played from a Bagfile to the node. The node runs on a Mac with no ROS2 installation.

    At the same time, there is a ROS1 node of a similar C++ LIO node running in a container on a Linux Laptop, that gets the exact same data as the Rust node at the same time. The Bagfile data is only published when our Rust node reaches some checkpoint in the code.

    Since the Rust node currently has a bug, that is not shared in the C++ node, we synchronise the Rust state to the C++ node, bypassing all the previous C++ pipeline to compare just one function. Then we go do the suspicious function where we think the problem is and see the difference of their result in our terminal. On top of that, we are defining a custom debug step on top of our synchronised ROS1 node, ROS2 node and Bagfile. We now can step through our pipeline, triggering the next iteration in our LIO by pressing <bkd>,</kbd> (comma) on the keyboard.


Let's have a quick look at the 3 features of Minot.

## Bagfile Querying
ss

## Variable Sharing
a
a
## Publish/Subscribe
g

## All 3 together
To demonstrate this, we look at the development of lidar inertial odometry.

