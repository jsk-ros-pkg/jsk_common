# filtered_relay.py

## Description

`rostopic echo` has a filter function, but it cannot publish a topic.
`topic_tools/transform` can format a topic and publish it, but it does not have the ability to filter.
Transforming while filtering a topic is simple and powerful.
This node provides a filter and publish (i.e. relay) functions.

## Subscribing Topic

* `~input` (`rospy/AnyMsg`)

  Input message.

## Publishing Topic

* `~output` (`~output_type`)

  Output topic. You can specify `~output_type` param to publish topic.

## Parameters

* `~output_type`  (`String`, required)

  Message type of output_topic like `std_msgs/Float32`. This is the input for the [get_message_class](http://docs.ros.org/en/diamondback/api/roslib/html/python/roslib.message-module.html#get_message_class) function.

* `~filter`  (`String`, default: `None`)

  Condition of messages that match a specified Python expression.

  The Python expression can access any of the Python builtins plus: ``topic`` (the topic of the message), ``m`` (the message) and ``t`` (time of message).

  For example, ``~input`` topic is ``std_msgs/String`` and if you want to check whether a sentence is a ``hello``, you can do the following.

```bash
filter: m.data == 'hello'
```

  Note that, use escape sequence when using the following symbols ``<(&lt;)``, ``>(&gt;)``, ``&(&amp;)``, ``'(&apos;)`` and ``"(&quot;)`` in launch file.

* `~transform`  (`String`, default: `m`)

  Python expression that transform the input messages, which are given in the variable m. The default expression is `m`, which results in forwarding input (which can be a topic field) into output_topic.

* `~import`  (`List[String]`, default: `[]`)

  List of Python modules to import and use in the expression.

## Usage

```bash
$ rosrun jsk_topic_tools filtered_relay.py ~input:=/right_endeffector/wrench \
    ~output:=/right_endeffector/force_norm \
    _filter:='numpy.linalg.norm([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]) > 10.0' \
    _output_type:=geometry_msgs/WrenchStamped _import:="[geometry_msgs, numpy]"
```

## Example

The following example subscribe to `/right_endeffector/wrench` and only those with a force norm greater than 10 are published as `/right_endeffector/force_norm`.

```bash
roslaunch jsk_topic_tools sample_filtered_relay.launch
```
