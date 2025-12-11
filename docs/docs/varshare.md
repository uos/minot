# Variable Sharing

When developing your node and choosing a tool to help you with it, you don't want to run through a huge setup or change your existing code a lot. The tool should adapt and move out of your way while still solving your problem.

In this case, the problem is transparency. We want to know what happens in our systems at any point and see if our code does what we think it should do. That is the reason we write tests. For larger ROS networks, this becomes a lot more challenging.

This is where *librat* comes in. It is a dependency free library to synchronise data between processes.


We'll start with a descriptive file. To learn more about the used language, read the [Bagfile Query](./bagquery.md) chapter.

## `rule!`

Define a strategy how a variable should be shared with a different Rat.


~~~awk title="Example"
rule! (myvar

  # Send myvar from rat1 to rat2, overwriting rat2
  rat1 -> rat2

  # rat3 overwrite both variables of rat1 and rat2
  rat1, rat2 <- rat3

  # Both rats send their myvar to the Minot TUI where they can be compared with each other
  blue == yellow

  # rat1 sends it myvar to the Minot TUI for displaying it
  rat1 -> LOG
)
~~~

!!! info "Formal Definition"
    ~~~ title="Definition"
    rule! (<Var>
      <Rats> <Operator> <Rats>
      ...
    )
    ~~~

    ~~~ title="Definition Primitives"
    <Rats> := (String | LOG) [, <Rats>]
    <Operator> := -> | <- | == | =
    ~~~


    Even though both sides of the operator can be an Array, the source side of it can never have more than one Rat. A line like `rat_a, rat_c -> rat_b` is not valid. `rat_b -> rat_a, rat_c` works and is interpreted as `rat_b` sending its variable to both `rat_a` and `rat_c`.


Every variable inside a `rule!` block is expected to resolve to a connected Rat.

The `->` operator signalises sending the variable defined at the head from the left side of the operator to the right side. The data of the Rats on the right are overwritten. These operations are synchronised, meaning they wait for each other and block your thread.

Sending from right to the left is signalised with `<-` or `=`. These operators are redundant but they can make a difference in readability so we offer both options.

You can also send everything to the Minot TUI to compare them with the `==` operator. This is just a shorter version of sending assigning each Rat to the hardcoded TUI name `LOG`. You can always add the `LOG` at any receiver list to also get the value inside the Comparison Window in Minot TUI.

### Setup

#### Standalone Coordinator

Variable Sharing only needs the Coordinator, which can run standalone without the TUI. The binary takes a Ratslang file with Rules. It waits for every Rat in that Rule to be connected to the network before un(b)locking all Rats to continue their normal flow after intialisation.

~~~bash title="Standalone Coordinator"
minot-coord ./rules.mt
~~~

#### Minot TUI with embedded Coordinator

When using the TUI with integrated Coordinator (the default when building Minot), the Rules should not be defined in the file you give to the TUI at startup. You need to set the `_rules` variable to a Path or String that is relative to the Ratslang file you are defining the variable in.

This is basically a specialized include statement but it needs to be explicitely stated because there is a difference of where you define Rules.

~~~awk title="Defining Rules when running with TUI"
_rules = ./my_rules.mt
~~~

You still can define Rules in the same file you write your Bagfile Query code. But the effect will be vastly different because Rules are also dynamic. You can delete all of them at runtime and add them again by evaluating lines within Minot TUI. You can read more about it [here](./tui.md).

??? info "Why is that needed? Race Conditions"

    If you combine both, Bagfile Queries and Rule definitions into one file when running the TUI with Coordinator, it effectively activates a datarace.

    The Coordinator sees all needed Rats but then the TUI sends a command to delete them all because the TUI interprets the Rules as something new at runtime. In the meantime the Rats may already be connected but there is no guarantee.

    If a Rat asks the Coordinator what to do for a specific variable now, the coordinator does not have Rules for the Rat and tells it to continue. Then the Rule from the TUI comes in and the other Rat asks what to do at the same variable that the other Rat just skipped. The coordinator sees the new Rule and tells the Rat to wait or send its variable to the other one that already asked. Both Rats aren't synchronised anymore and deadlocks occur where both Rats wait for each other.



## Library

The library for sharing variables is just called Rat. It is a Rust library with C bindings and built for minimal intrusion. See the [installation guide](./installation.md) for how to setup the technical side in your project.

There are 3 functions.

=== "C"
    ~~~c
    // Connect to a reachable Coordinator
    int rat_init(const char *node_name, int timeout_secs); // timeout <= 0 blocks until connected

    // Read/Write a 2D column-major array
    int rat_bacon_f32(const char *variable_name, float *data, size_t rows, size_t cols);

    // All possible types for bacon in C:
    // u8  -> char on ARM; unsigned char on non-ARM
    // f32 -> float
    // f64 -> double
    // i32 -> int

    // Disconnect from the network
    int rat_deinit(); // optional
    ~~~

=== "Rust"
    ~~~rust
    use rat::*;

    // Connect to a reachable Coordinator
    fn init(
      node_name: &str,
      timeout: Option<std::time::Duration>,
      runtime: Option<Arc<tokio::runtime::Runtime>>,
    ) -> anyhow::Result<()>;


    // Read/Write any (De)Serializable Type
    fn bacon<T>(
        variable_name: &str,
        data: &mut T,
        variable_type: VariableType,
    ) -> anyhow::Result<()>;

    // Disconnect from the network
    fn deinit() -> anyhow::Result<()>;
    ~~~

    !!! info "VariableType Argument in `fn bacon`"

        This argument is currently needed for the C interop. So if you want to send a matrix to C or Mino TUI, the underlying datatype for the matrix must be specified. If you share variables with other Rusty Rats, you can set `VariableType::default()`.

The [example section](https://github.com/uos/minot/tree/main/mt_rat/examples) on GitHub should give you enough code to use the library in your project without any more explanations.

For a general description of data, the shared datatype in C is a 2D Matrix, which is also the only supported format for the Minot TUI Variable Viewer. If you want to share more abstract data than  1D Vectors or 2D Arrays, you'll need to encode the data yourself. Either into a different typed Matrix or into raw bytes (unsigned char on non-ARM). The `rows` argument is then `0` and `cols` is the length of your buffer.

On the Rust side, there is a type called `NetArray` to assure C and TUI viewer compatibility. If you don't need that compatibility, you can also transport any type with automatic serialization by deriving `Serialize`, `Deserialize` and `Archive` from the powerful [`rkyv`](https://crates.io/crates/rkyv) crate – just like in the next feature: [Native Pub/Sub](./pubsub.md).

