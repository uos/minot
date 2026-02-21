# Sharing Memory Synchronously

One of the main tasks of the ROS Pub/Sub system (excluding non-sensor pipelines with actions and service) is sharing a new value of something. At development time, when accuracy is at the highest priority, this async model might be in your way.

With *librat*, you can share variables synchronously over the network. The library is written in Rust but targeted at C. It's built for the absolute minimal user footprint. You initialise it once and then give it any variables to cling to without changing anything else in your existing code. Just look at the [header file](https://github.com/uos/minot/blob/main/mt_rat/rat.h). You then explain the routes to the Minot TUI with intuitive syntax.

Variable sharing is a powerful building block and it can easily be used outside of the ROS context. Click [here](../features/varshare.md) to learn more.

!!! tip "From Comparing to full E2E Testing"

    By just writing a "proc1 == proc2", and adding 4 lines of code in total, you can check the equality of a variable from two different processes (proc1, proc2) that don't need to share the language or system. Comparing is just syncing both variables with the TUI.

    This also effectively describes a unit test over the network. And together with using checkpoints as trigger for a Bagfile publish, Minot effectively gives you an entire E2E testing framework for ROS networks.

## Quickstart

We will need [a modern Rust compiler](https://www.rust-lang.org/tools/install) for this example.

Build and run the libraries and the Coordinator.

~~~bash
git clone https://github.com/uos/minot && cd minot
cargo build

./target/debug/minot-coord mt/varshare_demo.mt
~~~

Start a second terminal to run `rat1` in Rust.
~~~bash
cargo run --example rat1
~~~

Start a third terminal to build and run `rat2` in C.

~~~bash
gcc rat/examples/rat2.c -o rat2 -L./target/release -l:librat.a -lm -Wl,-z,noexecstack

./rat2

# rat1 and rat2 terminate successfully.
# Look at the source code of the C file you just compiled.
# rat1 just flipped the 0 from rat2 to 1.

# Clean up
cd .. && rm -rf minot
~~~

Notably, this example does not even run the Minot binary but just a minimal subset for network communication.

