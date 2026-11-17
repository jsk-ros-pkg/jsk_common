# jsk_common

[![ROS 1](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/config.yml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/config.yml)
[![ROS 2](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/ros2.yaml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_common/actions/workflows/ros2.yaml)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](jsk_common/package.xml)

[![jazzy](https://img.shields.io/ros/v/jazzy/jsk_common)](https://index.ros.org/r/jsk_common/#jazzy)
[![kilted](https://img.shields.io/ros/v/kilted/jsk_common)](https://index.ros.org/r/jsk_common/#kilted)
[![lyrical](https://img.shields.io/ros/v/lyrical/jsk_common)](https://index.ros.org/r/jsk_common/#lyrical)
[![noetic](https://img.shields.io/ros/v/noetic/jsk_common)](https://index.ros.org/r/jsk_common/#noetic)
[![ROS One](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/jsk-ros-pkg/jsk_common/badges/ros_one.json)](https://ros.packages.techfak.net/)

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

"part" is a subset of the nodes of ROS 1: the README of each package (linked above) lists its nodes, launch files
and usage, and which of them are on ROS 2. There are no ROS 2 launch files yet: run the nodes with
`ros2 run <package> <executable>`; the `.launch` files are of ROS 1.

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
# ROS 2: skip the packages of ROS 1 only, which need catkin
colcon build --packages-skip jsk_network_tools jsk_tilt_laser multi_map_server virtual_force_publisher
# ROS 1
catkin build
```
