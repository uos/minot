# Bagfile Query

While developing ROS nodes, Bagfiles can help you a lot for reproducability. With Minot you can query that recording and control each message that is published to your nodes. Bagfile Query is not built for a long stream of data. The idea is to grab specific chunks that fit some query and send them via the Coordinator (like a ROS1 Master) to all connected nodes that care about publishing Bagfile data. It is not meant to be a replacement for `ros2 bag play`. If you *just want to play a Bagfile*, we recommend the usual ROS tooling.

To establish the power of querying, we will start off by implementing `bag play` in Minot and then go from there.


First off you need the [Minot TUI binary](installation.md#minot-tui). It expects a Ratslang file (`.rl`) as the first argument. You will need the line numbers of that file while interacting with Minot TUI at the same time so keep them both open side by side.

!!! info "Ratslang"

    Ratslang is a minimal configuration language that inherently handles units. You should be able to read and speak it fluently by reading the 20 line example [here](https://github.com/stelzo/ratslang/blob/main/README.md).

Minot extends Ratslang with some functions that are built right into the language. They are used for querying the bagfile.

---

## `reset!`

Load a Bagfile set the cursor to the beginning.

This function requires a path to a Bagfile (directory name). The path is relative to your current working directory where you started the Minot TUI.

~~~ title="Definition"
reset! <Path>
~~~

~~~awk title="Example"
reset! ./mybag
~~~

!!! warning

    At the moment, we only support the MCAP storage backend. It is the default for Rosbags since ROS2 Iron Irwini. For older Bagfiles, follow the conversion guide [here](https://mcap.dev/guides/getting-started/ros-2).

## `pf!`

Play frames (messages) from the currently loaded Bagfile.

~~~ title="Definition Normal"
pf! <Topics> <Span> [<PlayingSpan> [<Trigger>]] 
~~~

~~~ title="Definition Conditional"
pf! <Topics>, <UntilTopics> <UntilCondition> [<PlayingSpan> [<Trigger>]] 
~~~

~~~ title="Definition Primitives"
<UntilTopics> := <Topics>
<Topics> := Array<TopicIdentifier> | <TopicIdentifier>

<PlayingSpan> := <RelativeSpan> | <Factor>
<Factor> := <Float>

<Span> := <AbsoluteSpan> | <RelativeSpan>
<AbsoluteSpan> := [<Time>]..[<Time>]
<RelativeSpan> := <Time>

<Trigger> := <TriggerVariable> [<TriggerMode>]
<TriggerVariable> := <String>
<TriggerMode> := d | f
~~~

The `<TriggerMode>` specifies dynamic dispatch with `d` and fixed with `f`. When not specified, dynamic is assumed. Dynamic dispatch means the Coordinator will ask the TUI to evaluate the query when reaching the variable. Fixed mode will evaluate the query first and publish the same messages everytime it reaches the variable.

`<TriggerVariable>` must correspondend to a variable set in the rules for sharing variables. Click [here](./varshare.md) for more information.

`<TriggerVariable>` is defined by the user using Ratslang. There is an existing struct format Minot is looking for. It looks like this:

~~~ title="Setting TopicIdentifier"
_bag.{

  yourchosenname.{
    _topic = <Path>
    _short = <String>
    _type = Cloud|Imu|Any
  }
  
  yournexttopic.{
    ...
  }

  ...
}
~~~

Every new namespace you define in `_bag` will be checked for `_topic`, `_short` and `_type`. You can interpret this as configuring an Array of Structs without having Structs in the language.

`yourchosenname` is a filler for any name you choose to call this TopicIdentifier. You can then use that name inside your `pf!` function.

When debugging something with fast iterations, it is often convenient to abbreviate your names. With `_short` you can give your identifier an alternative name.

`_type` is one the main interests for Minot when querying. It matches the given type to the message at the current cursor position. `_type` is one of "Cloud", "Imu" or "Any". They can easily be extended to more messages as needed. A type is necessary as soon as you want to publish messages outside of ROS2 installations. *Cloud* relates to `sensor_msgs/msg/PointCloud2` and *Imu* to `sensor_msgs/msg/Imu`.

*Any* will match every message. In the [installation](./installation.md), this behaviour is defined as *with or without any-type*. By specifying an exisiting type, Minot can use it outside of ROS2 land. For example in ROS1 or the the [Native Pub/Sub](./pubsub.md) library.

`_topic` allows for more detailed message matching. When you got multiple *Imu* topics in your Bagfile, this parameter becomes essential.

??? warning "Multiple Types and QoS per Topic"

    ROS2 allows many publishers to the same topic with different types and QoS settings. Minot assumes a simple environment, where a topic is always published by one publisher with one QoS. While multiple publishers work with Minot, it itself only uses a single publisher with one QoS, which still works for common multi-publisher systems like TF.

Conditional Queries are a special form of `<Span>` but it also disables some combinations that make no sense in this context so they will fail with an error. There is only one important consideration when using Conditionals: they are exclusive. So a query like `pf! x, y 1` will stop the cursor right before the `y` topic. You can *consume* it then with any other query since they are inclusive. Like a fxied number: `pf! y 1`.

--- 
Now we got everything we need to implement the `ros2 bag play` command within Minot.

~~~awk title="ros2 bag play ./bagfile"
# Define our topic filter
_bag.any._type = Any

# Load the bagfile
reset! ./bagfile

# Play everything at original speed
pf! any .. 1.
~~~

!!! warning

    With this code, Minot will load the entire Bagfile into memory first, which is probably not want you want. This is why you should still use `ros2 bag play` for this use case -- but it beautifully demonstrates the flexibility of Minots query feature.

If you can understand this, you are ready for some more examples.

### Examples
~~~awk title="demo.rl"
_bag.{
  lidar.{
    _topic = "/velodyne_points"
    _type = Cloud
    _short = "l"
  }
  imu.{
    _topic = "/imu/data"
    _type = Imu
    _short = "i"
  }
}

# send all lidar frames between the time of 10s and 20s of the bagfile immediately
pf! l 10s..20s

# send all lidar and imu frames between the time of 10s and 20s of the bagfile when reaching var1
pf! [l, i] 10s..20s var1

# send all lidar frames between the time of 10s and 20s of the bagfile but slow down time so it sends it over 2 minutes
pf! l 10s..20s 2mins

# send the next imu frames until 2 lidar frames are encountered
pf! i, l 2

# send the next 10 lidar frames but spread them out over 20s
pf! l 10 20s

# send the next 10 seconds of lidar frames everytime a rat reaches var1
pf! l 10s var1

~~~

For information on how to execute the code in Minot TUI, click [here](./tui.md#bagfile-query).
