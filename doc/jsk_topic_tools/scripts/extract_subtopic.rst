static_transform_republisher.py
===============================


What is this?
-------------

This node can publish sub-field of input topic. (e.g. publish `geometry_msgs/Pose` msg from `msg.pose.pose` field of `/odom` topic with `nav_msgs/Odometry`)

Subscriber
----------

- `~input` (Anymessage)

  Input topic and subtopic field. ( If you want to use `msg.pose.pose` field of `/odom` topic, set this to `/odom/pose/pose` )

Publisher
---------

- `~output` (Type of subtopic)

  Output topic
