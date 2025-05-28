# Overview

Minot provides primitives for developing and validating stateful robot software.

TODO cool image of the tui with some rats code and lio.

When developing, we want to solve a small problem that plays nicely with the larger system. Modularity is one of the main goals of ROS. And while developing and integrating these modules, Minot is here to help.

!!! question "The Pain Leading to Minot"

    === "Message Timing"

        Does your system behave different each time you play a Bagfile until you find out it actually works when playing it a little bit slower?

    === "Ignore Networking"

        Have you ever asked yourself why you should care about QoS settings, out-of-order message pipelines or the dependency of ROS2 itself when developing a Proof of Concept?

    === "Reproducability"

        Did you ever develop a ROS Node with state that unexpectedly changes for some reason and now you try to reproduce it?

Minot is meant to be used at development time to give you full control over the incoming data. It gives you transparency for complex systems where thousands of small sensor data packets are processed each second. With Minot, you regain control and explainability over what's happening when developing and integrating your Robot.

**How?**

Minot is a collection of binaries and libraries. Some of them usable in any existing ROS1 or ROS2 node, others completely separate from any system. This allows cross-ROS communication using its own networking layer without adding dependencies. With minot you can write, test and debug ROS perception nodes without needing ROS on your system.

This is enabled by 3 features, we call them *primitives*. They can be used standalone but when combined, they enable superpowers.

## Bagfile Querying

With ROS you can record and play everything published to the ROS network. This enables reproducability for nodes and greatly improves the safety of developing real robots. Bagfiles are great! But the ROS tooling for playing the Bagfile is limited. It's still enough for most use cases but there are times where you need complete control over the data that is published to your subscribers. Some kind of query language for fine grained filtering and cursor control inside your bagfile for fast iterations. This is Bagfile Querying in Minot.

It introduces a small, embedded language and functions for maximal control over your node while keeping flexibility. For more information, visit the feature page [here](bagquery.html).

## Variable Sharing

One of the main tasks of the ROS Pub/Sub system (excluding non-sensor with actions and service) is sharing a new value of something. At development time, when accuracy is at the highest priority, this async model might be in your way.

With *librat*, you can share variables synchronously over the network. It is library written in Rust but targeted at C. It's built for the absolute minimal user footprint. You initialise it once and then give it any variables to cling to without changing anything else in your existing code. Just look at the [header file](https://github.com/uos/minot/blob/main/rat/rat.h). You then explain the routes to the Minot TUI with intuitive syntax.

Variable sharing is a powerful building block and it can easily be used outside of the ROS context. Click [here](varshare.html) to learn more.

!!! tip "From Comparing to full E2E Testing"

    By just writing a "proc1 == proc2", and adding 4 lines of code in total, you can check the equality of a variable from two different processes (proc1, proc2) that don't need to share the language or system. Comparing is just syncing both variables with the TUI.

    This also effectively describes a unit test over the network. And together with using checkpoints as trigger for a Bagfile publish, Minot gives you an entire E2E testing framework.



## Native Publish/Subscribe

The Pub/Sub model has proven itself among ROS developers. It's an intuitive way to exchange time series data. But sometimes adding the entire ROS stack with all its solutions for problems outside of the domain of the problem your node tries to solve seems like overkill.

For these cases, the native Publisher and Subscriber Rust library *ratpub* can help. By building on the same concepts as Variable Sharing, it removes ROS from the dependency list at development time if you only use that API from ROS. Read more about it [here](pubsub.html).

## Synergies

While useful for some cases on their own, together they become a toolbelt for understanding complex stateful systems.

To demonstrate this, we will take a look at a real Minot workflow for lidar inertial odometry.

TODO

!!! info "What is going on?"

    You are looking at a Lidar Inertial Odometry ROS2 node written in Rust. For IMU initialisation, just the needed frames are played from a Bagfile to the node. The node runs on a Mac with no ROS2 installation.

    At the same time, there is a ROS1 node of a similar C++ LIO node running in a container on a Linux Laptop, that gets the exact same data as the Rust node at the same time. The Bagfile data is only published when our Rust node reaches some checkpoint in the code.

    Since the Rust node currently has a bug, that is not shared in the C++ node, we synchronise the Rust state to the C++ node, bypassing all the previous C++ pipeline to compare just one function. Then we go do the suspicious function where we think the problem is and see the difference of their result in our terminal. On top of that, we effectively built our own debug pipeline step on top of our synchronised ROS1 node, ROS2 node and Bagfile. We can now step to the next checkpoint, triggering the next iteration in our LIO by pressing <bkd>,</kbd> (comma) on the keyboard. We essentially built a gdb over the network by adding just a few lines of code to our existing nodes.
