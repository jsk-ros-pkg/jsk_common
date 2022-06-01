sanity\_diagnostics.py
==============

## What is this

This node publishes essential robot topic and node status to `/diagnostics`.

## Publishing Topics

* `/diagnostics` (`diagnostic_msgs/DiagnosticStatus`)

    Diagnostics topic

## Subscribing Topics

* topics specified in the `~sanity_targets` yaml file

## Parameter

* `~sanity_targets` (`String`, default: `/var/lib/robot/sanity_targets.yaml`)

    Yaml file which contains topics and nodes to be monitored.

    Sample yaml format for fetch:

    ```yaml
    topics:
      - /audio
      - /base_scan
      - /battery_state
      - /edgetpu_human_pose_estimator/output/image
      - /edgetpu_object_detector/output/image
      - /gripper/imu
      - /head_camera/depth/image_raw
      - /head_camera/rgb/image_raw
      - /imu
      - /insta360/image_raw
      - /joint_states
      - /tf

    nodes:
      - /amcl
      - /auto_dock
      - /gripper_driver
      - /head_camera/head_camera_nodelet_manager
      - /move_base
      - /move_group
      - /respeaker_node
      - /robot_driver
      - /roswww
    ```

* `~duration` (`Float`, default: `1`)

    Duration in which sanity is checked and `/diagnostics` is published.

## Usage

Visualize topic and node diagnostics.

```bash
roslaunch jsk_tools sample_sanity_diagnostics.launch
```

![sanity_diagnostics](https://user-images.githubusercontent.com/19769486/168039418-0171a6dc-9507-436d-a9a6-13a2c7f52327.png)
