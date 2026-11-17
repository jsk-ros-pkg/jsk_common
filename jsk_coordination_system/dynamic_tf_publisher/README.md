# dynamic_tf_publisher

Publish TF transforms that can be added, changed, re-parented and removed at runtime through services.
`tf_publish.py` keeps a set of transforms and broadcasts them periodically; `tf_set.py` is a command line client to set one,
and `tf_reconfigure_publisher.py` broadcasts a single transform whose values are tuned through parameters
(dynamic_reconfigure on ROS 1).

## ROS 1 / ROS 2

The package builds for both ROS 1 (catkin) and ROS 2 (ament_cmake); `CMakeLists.txt` branches on `$ENV{ROS_VERSION}`.

| Item | ROS 1 | ROS 2 |
|---|---|---|
| `tf_publish.py` | yes | yes |
| `tf_set.py` | yes | yes |
| `tf_reconfigure_publisher.py` | yes | yes |
| Services `SetDynamicTF`, `AssocTF`, `DissocTF`, `DeleteTF` | yes | yes |
| dynamic_reconfigure config `TfParameter` | yes | - (plain parameters) |
| `samples/sample.launch` | yes | - |

There are no ROS 2 launch files; use `ros2 run dynamic_tf_publisher <exe>`.

## tf_publish.py

Node name `tf_publish_server`. Broadcasts all registered transforms to `/tf`.

Services:

* `/set_dynamic_tf` (`dynamic_tf_publisher/SetDynamicTF`) -- add or update the transform `cur_tf`
  (`header.frame_id` -> `child_frame_id`), published every `1 / freq` seconds.
* `/assoc_tf` (`dynamic_tf_publisher/AssocTF`) -- re-parent `child_frame` to `parent_frame`, keeping its current pose
  (e.g. attach an object frame to a gripper).
* `/dissoc_tf` (`dynamic_tf_publisher/DissocTF`) -- undo `assoc_tf` for `frame_id`, re-attaching it to its original parent.
* `/delete_tf` (`dynamic_tf_publisher/DeleteTF`) -- stop publishing the frame `header.frame_id`.
* `/publish_tf` (`std_srvs/Empty`) -- publish the transforms immediately.

ROS 1 only:

* Also publishes the transforms on `~tf` (`tf2_msgs/TFMessage`).
* `~use_cache` (Bool, default `true`) -- store the transforms on the parameter server and restore them when the node restarts.
* `~check_update` (Bool, default `false`) -- publish only updated transforms, and all of them every `~check_update_sleep_time`
  (default `1.0` s).
* Exits if the services are already advertised by another `tf_publish.py`.

On ROS 2, the transforms are broadcast at a fixed 1 Hz (the `freq` field of `SetDynamicTF` is not used for the rate).

## tf_set.py

Calls `/set_dynamic_tf`.

```
tf_set.py x y z (yaw pitch roll | qx qy qz qw) parent child period_msec
```

The rotation is either three Euler angles applied in z-y-x order or a quaternion; the last argument is the publishing
period in milliseconds.

## tf_reconfigure_publisher.py

Broadcasts one transform `parent_frame_id` -> `frame_id` at 10 Hz with parameters `frame_id`, `parent_frame_id`,
`x`, `y`, `z`, `roll`, `pitch`, `yaw`. On ROS 1 they are changed with dynamic_reconfigure (`TfParameter.cfg`),
on ROS 2 with `ros2 param set`.

## Usage

ROS 1:

```bash
roslaunch dynamic_tf_publisher sample.launch    # tf_publish.py and four tf_set.py calls (map -> desk -> cup, map -> robot -> hand)

rosrun dynamic_tf_publisher tf_publish.py
rosrun dynamic_tf_publisher tf_set.py 1 0 1 0 0 0 robot hand 1000
rosservice call /assoc_tf "{parent_frame: hand, child_frame: cup}"
rosservice call /dissoc_tf "{frame_id: cup}"
```

ROS 2:

```bash
ros2 run dynamic_tf_publisher tf_publish.py
ros2 run dynamic_tf_publisher tf_set.py 1 0 1 0 0 0 robot hand 1000
ros2 service call /delete_tf dynamic_tf_publisher/srv/DeleteTF "{header: {frame_id: hand}}"

ros2 run dynamic_tf_publisher tf_reconfigure_publisher.py --ros-args \
  -p parent_frame_id:=map -p frame_id:=marker -p x:=1.0
```

## Launch files (ROS 1)

* `samples/sample.launch` -- starts `tf_publish.py` and sets `map -> desk`, `desk -> cup`, `map -> robot` and `robot -> hand`
  with `tf_set.py`.
