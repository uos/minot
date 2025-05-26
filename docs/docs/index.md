# Overview

Minot provides primitives for developing and validating stateful robot software.

Did you ever delevop a ROS Node with state that unexpectedly shifts for some reason and now you try to reproduce it?

Does your system behave differently each time you play a bagfile until you find out it actually works when playing the bagfile a little bit slower?

Have you ever tried to develop a new algorithm for just a small subproblem of a larger system and now you need to use this language and tools of the old method?

Have you ever asked yourself why you should care about QOS Settings, out-of-order message pipelines or the dependency of ROS2 itself when developing a Proof of Concept?

Minot was the result out of this painpoints. When developing, we want to solve a small problem that plays nicely with the larger system. Modularity is one of the main goals of ROS. And while developing and integrating these modules, Minot is here to help.

It is meant to be used at development time to give you full control over the incoming data and gives you transparency for complex systems where thousands of small sensor data packets are processed each second. With Minot, you regain control and explainability over what's happening when testing your system.

*How?*

We use the word "primitives" here. Minot is a collection of binaries and libraries. Some of them usable in any existing ROS1 or ROS2 node, others


1. Bagfile Querying
1. Distributed Variable Sharing and Comparing
1. Native Publish/Subscribe

These can be used standalone but the power of Minot comes when these 3 features are used in combination with each other.

## Bagfile Querying
ss

## Variable Sharing
a
a
## Publish/Subscribe
g

## All 3 together
To demonstrate this, we look at the development of lidar inertial odometry.

