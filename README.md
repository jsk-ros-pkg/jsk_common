# jsk_common

[![ROS 1](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/config.yml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/config.yml)
[![ROS 2](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/ros2.yaml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/ros2.yaml)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](jsk_common/package.xml)

[![jazzy](https://img.shields.io/ros/v/jazzy/jsk_common)](https://index.ros.org/r/jsk_common/#jazzy)
[![kilted](https://img.shields.io/ros/v/kilted/jsk_common)](https://index.ros.org/r/jsk_common/#kilted)
[![lyrical](https://img.shields.io/ros/v/lyrical/jsk_common)](https://index.ros.org/r/jsk_common/#lyrical)
[![noetic](https://img.shields.io/ros/v/noetic/jsk_common)](https://index.ros.org/r/jsk_common/#noetic)
[![ROS One](https://img.shields.io/badge/ROS%20One-ros--one--jsk--*-22314E)](https://ros.packages.techfak.net/)

Common tools of the JSK Lab (the University of Tokyo) for ROS: topic relays and throttles, nodelet base classes, data
and rosbag tools, an image viewer with drawing, a dynamic TF publisher, and development tools. The same source builds
for ROS 1 (catkin) and ROS 2 (ament); the packages switch by `$ROS_VERSION`, and `jsk_ros1_ros2_compat` keeps the
code shared.

## Packages

| Package | ROS 1 | ROS 2 | What it is |
|---|:---:|:---:|---|
| [jsk_topic_tools](jsk_topic_tools) | ✓ | ✓ (part) | Topic relays, throttles and muxes, and the base classes of nodelets / lifecycle nodes |
| [jsk_tools](jsk_tools) | ✓ | ✓ (part) | Development and lab tools: roscore checks, test helpers, an audible warning of diagnostics |
| [jsk_data](jsk_data) | ✓ | ✓ (part) | Downloading and uploading data, recording and playing rosbags, collecting data |
| [jsk_rosbag_tools](jsk_rosbag_tools) | ✓ | ✓ | Converting and editing bags: to video and audio, merging, compressing images |
| [audio_video_recorder](audio_video_recorder) | ✓ | ✓ | Recording images and audio into a video file, synchronously (GStreamer) |
| [image_view2](jsk_ros_patch/image_view2) | ✓ | ✓ (part) | An image viewer which draws markers on the images and selects rectangles and points |
| [dynamic_tf_publisher](jsk_coordination_system/dynamic_tf_publisher) | ✓ | ✓ | Publishing TF frames which are set and changed by services |
| [jsk_ros1_ros2_compat](jsk_ros1_ros2_compat) | ✓ | ✓ | The ROS 1 / ROS 2 compatibility layer (C++ headers, Python module, CMake helpers) of these packages |
| [jsk_network_tools](jsk_network_tools) | ✓ | | Topics over low- and high-bandwidth links (silverhammer), network and Wi-Fi status |
| [jsk_tilt_laser](jsk_tilt_laser) | ✓ | | Point clouds of a tilting laser |
| [multi_map_server](jsk_ros_patch/multi_map_server) | ✓ | | A map_server serving several maps |
| [virtual_force_publisher](virtual_force_publisher) | ✓ | | The force of the end effector estimated from the joint torques |
| [jsk_common](jsk_common) | ✓ | ✓ | The metapackage |

"part" is a subset of the nodes of ROS 1, as below. There are no ROS 2 launch files yet: run the nodes with
`ros2 run <package> <executable>`; the `.launch` files are of ROS 1.

### jsk_topic_tools

- ROS 1: nodelets `jsk_topic_tools/LightweightThrottle`, `ConstantRateThrottle`, `SynchronizedThrottle`, `Relay`,
  `MUX`, `Passthrough`, `Block`, `Snapshot`, `HzMeasure`, `VitalCheckerNodelet`, ...; `standalone_complexed_nodelet`
  (many nodelets in one process), `topic_buffer_server` / `topic_buffer_client` (topics over a slow link); scripts
  such as `topic_compare.py`, `filtered_relay.py`, `static_image_publisher.py`, `tf_to_pose.py`, `bag2csv.py`; the
  base classes `ConnectionBasedNodelet` and `DiagnosticNodelet`. Launch: `standalone_complexed_nodelet_sample.launch`,
  `tf_buffer_server.launch` / `tf_buffer_client.launch`, `passthrough_sample.launch`, ...
- ROS 2: `connection_based_lifecycle_node` (the lifecycle counterpart of `ConnectionBasedNodelet`), the component
  `jsk_topic_tools::StringRelayLifecycleNode` (`string_relay_lifecycle_node`), `transform_merger`,
  `static_transform_pose_stamped`, `filtered_relay.py` and `array_to_heatmap.py`.

```bash
# ROS 1: /camera/image_raw at 1 Hz on /lightweight_throttle/output
rosrun nodelet nodelet standalone jsk_topic_tools/LightweightThrottle __name:=lightweight_throttle \
  ~input:=/camera/image_raw _update_rate:=1.0
# ROS 2: the non-empty messages of /chatter on /filtered_relay/output
ros2 run jsk_topic_tools filtered_relay.py --ros-args -r ~/input:=/chatter \
  -p output_type:=std_msgs/String -p filter:="m.data != ''"
```

### jsk_tools

- ROS 1: `roscore_check.py`, `roscore_monitor.py`, `check_host_sanity.py`, `audible_warning.py` (speaks the errors
  of the diagnostics), `battery_capacity_summary.py`, `launchdoc-generator.py`, `rostool-alias-generator.py`,
  `run_tmux`, and the rostest helpers `test_topic_published.py`, `test_rosparam_set.py`, `test_stdout.py`. Launch:
  `local_pc_monitor.launch`, `record_axis_camera.launch`.
- ROS 2: `audible_warning.py` and `battery_capacity_summary.py`.

### jsk_data

- ROS 1: the `jsk_data` command (getting and putting data), `data_collection_server.py` (saving the synchronized
  topics on a request), `rosbag_always.py`, `robot_description_loader.py` / `robot_description_saver.py`. Launch:
  `common_record.launch`, and the record and play launches of the robots (`pr2_record.launch` /
  `pr2_play.launch`, `baxter_*`, `fetch_*`, `hrp2_*`, ...).
- ROS 2: the `jsk_data` command, `data_collection_server.py` and `camera_coords_change_trigger.py`.

### jsk_rosbag_tools

ROS 1 and ROS 2 (rosbag2): `bag_to_video.py` (the images, with the audio, to a video), `bag_to_audio.py`,
`video_to_bag.py`, `merge.py`, `compress_imgs.py`, `tf_static_to_tf.py`, `add_heatmap.py`.

### audio_video_recorder

ROS 1 and ROS 2: `audio_video_recorder`. Launch (ROS 1): `audio_video_recorder.launch`.

### image_view2

- ROS 1: `image_view2` (`ImageMarker2` markers, mouse ROI), `points_rectangle_extractor`, `scale_interaction.py`.
- ROS 2: `image_view2`.

### dynamic_tf_publisher

ROS 1 and ROS 2: `tf_publish.py` (the TF broadcaster), `tf_set.py`, `tf_reconfigure_publisher.py`.

### ROS 1 only

- jsk_network_tools: `silverhammer_lowspeed_streamer.py` / `silverhammer_lowspeed_receiver.py`,
  `silverhammer_highspeed_*`, `heartbeat_master.py`, `network_status.py`, `wifi_status.py`. Launch:
  `*_silverhammer_*_sample.launch`, `wifi_status.launch`.
- jsk_tilt_laser: `tilt_laser_assembler.py`, `dynamixel_joint_state_publisher.py`. Launch: `jsk_laser_pipeline.launch`,
  `multisense_laser_pipeline.launch`, `dynamixel.launch`, ...
- multi_map_server: `multi_map_server`.
- virtual_force_publisher: `virtual_force_publisher`. Launch: `virtual_force_publisher.launch` (and of the left,
  the right and both arms).

## Install

```bash
# ROS 2 (jazzy, kilted, lyrical, rolling) and ROS 1 noetic
sudo apt install ros-$ROS_DISTRO-jsk-common
# ROS One (https://ros.packages.techfak.net)
sudo apt install ros-one-jsk-common
```

From source:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/jsk-ros-pkg/jsk_common.git
rosdep install -y -r --from-paths . --ignore-src
cd ~/ws
colcon build  # ROS 2, or catkin build for ROS 1
```

## Community

JSK Lab members can use the [Slack](https://jsk-robotics.slack.com) of the lab (with the imi address).
