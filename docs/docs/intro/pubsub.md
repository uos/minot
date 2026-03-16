# Pub/Sub without ROS

The Pub/Sub model has proven itself among ROS developers. It's an intuitive way to exchange time series data. But sometimes adding the entire ROS stack with all its solutions for problems outside of the domain of the problem your node tries to solve just seems like overkill.

For these cases, the native Rust library *mt_pubsub* can help. By building on the same concepts as Variable Sharing, it removes ROS from the dependency tree at development time if you only use the pub/sub API from ROS.

## Quickstart

~~~bash title="Pub/Sub without ROS"
# Clone the example nodes
git clone https://github.com/stelzo/mt_pubsub-demo && cd mt_pubsub-demo

# Run the nodes directly! 
# (The coordinator will automatically start in the background if none is found)
cargo run --bin node1
~~~

Start a second terminal to run another node.
~~~bash
cargo run --bin node2
~~~

Exit everything with Ctrl+C. Then cleanup everything we created: `cd .. && rm -r mt_pubsub-demo`.

Find out how these nodes work and how to write them [here](../features/pubsub.md).


