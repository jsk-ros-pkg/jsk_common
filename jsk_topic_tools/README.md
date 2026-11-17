# jsk_topic_tools

Utilities to relay, throttle, filter, buffer and inspect ROS topics.
The package provides a set of nodelets (lazy relays and throttles, mux, passthrough, Hz measurement, ...),
Python nodes and command line tools, and base classes for writing nodes that subscribe to their inputs
only while their outputs are subscribed (`ConnectionBasedNodelet`, `DiagnosticNodelet` in C++,
`ConnectionBasedTransport` in Python, and `ConnectionBasedLifecycleNode` on ROS 2).

## ROS 1 / ROS 2

The package builds for both ROS 1 (catkin) and ROS 2 (ament_cmake); `CMakeLists.txt` branches on `$ENV{ROS_VERSION}`.

| Item | ROS 1 | ROS 2 |
|---|---|---|
| Nodelets `jsk_topic_tools/{ConstantRateThrottle, LightweightThrottle, MUX, Relay, DeprecatedRelay, StealthRelay, Passthrough, Block, Snapshot, HzMeasure, VitalCheckerNodelet, StringRelay, SynchronizedThrottle}` (also as standalone executables `constant_rate_throttle`, `lightweight_throttle`, `mux`, `relay`, `deprecated_relay`, `stealth_relay`, `passthrough`, `block`, `snapshot`, `hz_measure`, `vital_checker`, `string_relay`, `synchronized_throttle`) | yes | - |
| `standalone_complexed_nodelet`, `topic_buffer_server`, `topic_buffer_client` | yes | - |
| `transform_merger`, `static_transform_pose_stamped` | yes | yes |
| C++ library `jsk_topic_tools` (`ConnectionBasedNodelet`, `DiagnosticNodelet`, utilities) | yes | - |
| C++ library `connection_based_lifecycle_node` (`ConnectionBasedLifecycleNode`) | - | yes |
| Component `jsk_topic_tools::StringRelayLifecycleNode` (executable `string_relay_lifecycle_node`) | - | yes |
| Python module `jsk_topic_tools` | yes | yes |
| `array_to_heatmap.py`, `filtered_relay.py` | yes | yes |
| Other scripts (`boolean_node.py`, `is_synchronized`, `rostopic_connection_list`, `synchronize_republish.py`, `static_tf_republisher.py`, `tf_to_pose.py`, `tf_to_transform.py`, `transform_wrench.py`, `pose_stamped_publisher.py`, `static_image_publisher.py`, `joint_state_pruner.py`, `topic_compare.py`, `topic_statistics.py`, `rosping_existence.py`, `bag2csv.py`) | yes | - |
| Launch files in `launch/` and `sample/` | yes | - |

There are no ROS 2 launch files; run the ROS 2 executables with `ros2 run jsk_topic_tools <exe>`.

## Nodelets (ROS 1)

All nodelets subscribe `~input` and publish `~output` unless noted. Most of them are lazy:
they subscribe `~input` only while `~output` has subscribers.

* **LightweightThrottle** -- like `topic_tools/throttle`, but subscribes the input only when the throttled topic is subscribed.
  Parameter `~update_rate` (default `1.0` Hz). [doc](../doc/jsk_topic_tools/lib/lightweight_throttle_nodelet.md)
* **ConstantRateThrottle** -- same interface as `LightweightThrottle`, but publishes at an accurate constant rate
  (re-publishing the buffered message if `~update_rate` is higher than the input rate). [doc](../doc/jsk_topic_tools/lib/constant_rate_throttle_nodelet.md)
* **SynchronizedThrottle** -- throttles several topics (`~topics`, up to 8) synchronously and advertises them with `~suffix`
  (default `throttled`). Parameters `~update_rate`, `~approximate_sync`, `~use_wall_time`, ... [doc](../doc/jsk_topic_tools/lib/synchronized_throttle_nodelet.rst)
* **Relay** -- like `topic_tools/relay`, with lazy mode and `~latch`; `~always_subscribe` disables the lazy mode. [doc](../doc/jsk_topic_tools/lib/relay_nodelet.md)
* **StealthRelay** -- relays `~input` only while `~monitor_topic` (default: `~input`) is subscribed by other nodes, i.e. subscribes
  without incrementing the subscriber count. [doc](../doc/jsk_topic_tools/lib/stealth_relay_nodelet.md)
* **Passthrough** -- relays only for a given duration. Services `~request`, `~stop` (`std_srvs/Empty`) and
  `~request_duration` (`jsk_topic_tools/PassthroughDuration`); parameter `~default_duration` (default `10.0` s). [doc](../doc/jsk_topic_tools/lib/passthrough_nodelet.md)
* **HzMeasure** -- publishes the rate of `~input` as `std_msgs/Float32` on `~output` and a diagnostic status.
  Parameters `~measure_time`, `~message_num`, `~warning_hz`. [doc](../doc/jsk_topic_tools/class/hz_measure_nodelet.md)
* **VitalCheckerNodelet** -- checks whether `~input` is alive and publishes the status to `/diagnostics`.
* **MUX** -- nodelet version of `topic_tools/mux`: selects one of `~topics`; services `~select`, `~add`, `~delete`, `~list_topics`,
  the selected topic is published on `~selected`.
* **Block** -- makes a chain of non-lazy nodes lazy: relays `~input` to `~input_original` and `~output_original` to `~output`,
  subscribing only while `~output` is subscribed (checked at `~check_rate`).
* **Snapshot** -- publishes a single `~input` message to `~output` each time the `~request` service (`std_srvs/Empty`) is called.
* **StringRelay**, **DeprecatedRelay** -- `std_msgs/String` relay, and a relay that warns that the topic is deprecated.

`standalone_complexed_nodelet` is a nodelet manager that loads the nodelets listed in the `~nodelets` (and `~nodelets_0` ... `~nodelets_99`)
parameters itself, avoiding the unreliable bond between manager and loader processes.
See [standalone_complexed_nodelet.rst](../doc/jsk_topic_tools/lib/standalone_complexed_nodelet.rst) and
[sample/standalone_complexed_nodelet/simple.launch](sample/standalone_complexed_nodelet/simple.launch).

## Nodes and scripts

* **filtered_relay.py** (ROS 1 / ROS 2) -- relays `~input` to `~output` only when the Python expression `~filter` holds,
  optionally transforming it with `~transform`. `~output_type` is required. [doc](../doc/jsk_topic_tools/scripts/filtered_relay.md)
* **array_to_heatmap.py** (ROS 1 / ROS 2) -- converts a numeric `std_msgs/*MultiArray` or single-channel `sensor_msgs/Image`
  into a colored `bgr8` heatmap image. Parameters `~cell_size`, `~colormap`, `~min_value`, `~max_value`, ... [doc](../doc/jsk_topic_tools/scripts/array_to_heatmap.rst)
* **transform_merger** (ROS 1 / ROS 2) -- subscribes `/tf`, keeps the latest transform per child frame and publishes them
  together on `/tf_merged` at `~loop_hz` (default `1.0`). The `transform_config/ignore_tf` list is ROS 1 only.
* **static_transform_pose_stamped** (ROS 1 / ROS 2) -- `static_transform_pose_stamped x y z yaw pitch roll from_topic to_topic`
  applies a fixed offset to every `geometry_msgs/PoseStamped` on `from_topic` and publishes it on `to_topic`.
* **string_relay_lifecycle_node** (ROS 2) -- example of `ConnectionBasedLifecycleNode`: relays `std_msgs/String` from `~/input`
  to `~/output`, subscribing only while `~/output` is subscribed (parameters `always_subscribe`, `verbose_connection`).
* **topic_buffer_server** / **topic_buffer_client** (ROS 1) -- buffer topics over a slow network: the client republishes the
  server's topics with the `_buffered` suffix, either periodically (`~update_rate`, `~fixed_rate`) or on request via the `update` service.
* **boolean_node.py** -- `or`/`and`/`not`/`xor` of several boolean inputs. [doc](../doc/jsk_topic_tools/scripts/boolean_node.rst)
* **synchronize_republish.py** -- republishes `~topics` with exact/approximate synchronization. [doc](../doc/jsk_topic_tools/scripts/synchronize_republish.rst)
* **is_synchronized** -- checks whether the given topics are synchronized. [doc](../doc/jsk_topic_tools/scripts/is_synchronized.rst)
* **rostopic_connection_list** -- lists topic connections between hosts. [doc](../doc/jsk_topic_tools/scripts/rostopic_connection_list.md)
* **static_tf_republisher.py** -- republishes `/tf_static` recorded in a rosbag. [doc](../doc/jsk_topic_tools/scripts/static_tf_republisher.rst)
* **tf_to_pose.py**, **tf_to_transform.py** -- publish a transform between two frames as `PoseStamped` / `TransformStamped`.
  ([tf_to_pose](../doc/jsk_topic_tools/scripts/tf_to_pose.rst), [tf_to_transform](../doc/jsk_topic_tools/scripts/tf_to_transform.rst))
* **transform_wrench.py** -- transforms a `WrenchStamped` into `~target_frame_id`. [doc](../doc/jsk_topic_tools/scripts/transform_wrench.rst)
* **pose_stamped_publisher.py** -- publishes a static `PoseStamped`. [doc](../doc/jsk_topic_tools/scripts/pose_stamped_publisher.md)

## Base classes

* `jsk_topic_tools::ConnectionBasedNodelet` (C++, ROS 1) -- subscribes inputs only while outputs are subscribed;
  subclasses must call `onInitPostProcess()` at the end of `onInit()`. Being superseded by `nodelet_topic_tools::NodeletLazy`.
  [doc](../doc/jsk_topic_tools/class/connection_based_nodelet.md)
* `jsk_topic_tools::DiagnosticNodelet` (C++, ROS 1) -- `ConnectionBasedNodelet` that also publishes diagnostics (`~vital_rate`, `~use_warn`).
  [doc](../doc/jsk_topic_tools/class/diagnostic_nodelet.md)
* `jsk_topic_tools.ConnectionBasedTransport` (Python, ROS 1) -- the same pattern for rospy nodes; implement `subscribe()` / `unsubscribe()`
  and create publishers with `self.advertise()`. [doc](../doc/jsk_topic_tools/class/python_connection_based_transport.md),
  example [sample/simple_image_transport.py](sample/simple_image_transport.py)
* `jsk_topic_tools::ConnectionBasedLifecycleNode` (C++, ROS 2) -- `rclcpp_lifecycle::LifecycleNode` with the same lazy subscription
  behavior; see [src/string_relay_lifecycle_node.cpp](src/string_relay_lifecycle_node.cpp) for a minimal subclass.

## Usage

ROS 1:

```bash
# lazy throttle of an image topic to 1 Hz
rosrun nodelet nodelet standalone jsk_topic_tools/LightweightThrottle __name:=lightweight_throttle \
  ~input:=/camera/image_raw _update_rate:=1.0

# relay only the messages that match a condition
rosrun jsk_topic_tools filtered_relay.py ~input:=/chatter _output_type:=std_msgs/String _filter:="m.data != ''"
```

ROS 2:

```bash
ros2 run jsk_topic_tools filtered_relay.py --ros-args -r '~/input:=/chatter' \
  -p output_type:=std_msgs/String -p filter:="m.data != ''"

ros2 run jsk_topic_tools array_to_heatmap.py --ros-args \
  -r '~/input:=/sensor/pressure' -r '~/output:=/sensor/pressure/heatmap'

ros2 run jsk_topic_tools string_relay_lifecycle_node --ros-args -r '~/input:=/chatter' -r '~/output:=/chatter_relay'
ros2 run jsk_topic_tools transform_merger --ros-args -p loop_hz:=1.0
```

## Launch files (ROS 1)

* `launch/tf_buffer_server.launch` / `launch/tf_buffer_client.launch` -- merge `/tf` with `transform_merger` and relay it to a remote
  PC through `topic_buffer_client`; `robot_tf_buffer_server.launch` / `robot_tf_buffer_client.launch` additionally buffer pruned
  `/joint_states` and run `robot_state_publisher` on the client side.
* `launch/topic_buffer_server_sample.launch`, `launch/topic_buffer_client_sample.launch`, `launch/topic_buffer_client_param_sample.launch`
  -- `topic_buffer_server` / `topic_buffer_client` with `roscpp_tutorials` talker/listener.
* `launch/passthrough_sample.launch` -- `Passthrough` sample (see the usage in [passthrough_nodelet.md](../doc/jsk_topic_tools/lib/passthrough_nodelet.md)).
* `launch/standalone_complexed_nodelet_sample.launch` -- `standalone_complexed_nodelet` sample.
* `launch/vital_checker_sample.launch` -- `vital_checker` with `diagnostic_aggregator` and `rqt_robot_monitor`.
* `sample/sample_*.launch` -- samples for `boolean_node.py`, `ConnectionBasedTransport`, `ConstantRateThrottle`, `filtered_relay.py`,
  `static_tf_republisher.py`, `synchronize_republish.py` and `transform_wrench.py`.

## Documentation

Reference pages for each nodelet, script and class are in [doc/jsk_topic_tools](../doc/jsk_topic_tools/index.rst)
([lib](../doc/jsk_topic_tools/lib), [scripts](../doc/jsk_topic_tools/scripts), [class](../doc/jsk_topic_tools/class)).
