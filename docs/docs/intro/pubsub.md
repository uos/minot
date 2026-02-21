# Pub/Sub without ROS

The Pub/Sub model has proven itself among ROS developers. It's an intuitive way to exchange time series data. But sometimes adding the entire ROS stack with all its solutions for problems outside of the domain of the problem your node tries to solve just seems like overkill.

For these cases, the native Rust library *ratpub* can help. By building on the same concepts as Variable Sharing, it removes ROS from the dependency tree at development time if you only use the pub/sub API from ROS.

## Quickstart

~~~bash title="Pub/Sub without ROS"
# Clone the example nodes
git clone https://github.com/stelzo/ratpub-demo && cd ratpub-demo

# Install the Coordinator
pip install minot-cli

# Run it
minot-coord
~~~


Start a second terminal to run the a node.
~~~bash
cargo run --bin node1
~~~

Start a third terminal another one.
~~~bash
cargo run --bin node2
~~~

Exit everything with Ctrl+C. Then cleanup everything we created: `cd .. && rm -r ratpub-demo && pip uninstall minot-cli -y`.

Find out how this nodes works and how to write them [here](../features/pubsub.md).


