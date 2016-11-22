==================
tf_to_transform.py
==================


What is this?
=============

Specific TransformStamped publisher.
You specify parent_frame_id and child_frame_id, and this node publish `geometry_msgs::TransformStamped`.
You can define `--duration` (default=0.1).

Difference from tf_to_pose.py
=============================

TransformStamped msg carries parent_frame_id and child_frame_id.
Otherwise, PoseStamped msg only carries parent_frame_id.

Usage
=====

Command Line
------------

.. code-block:: bash

    $ rosrun jsk_topic_tools tf_to_transform.py -h
    usage: tf_to_transform.py [-h] [--duration DURATION]
                              [parent_frame_id] [child_frame_id]

    positional arguments:
      parent_frame_id       parent frame id
      child_frame_id        child frame id

    optional arguments:
      -h, --help            show this help message and exit
      --duration DURATION, -d DURATION
                            Duration [s]: default=1.0


Param
-----

* `~parent_frame_id` (String)

  Parent frame id

* `~child_frame_id` (String)

  Child frame id

* `~duration` (Float, default=1.0)

  Duration

Example
=======

.. code-block:: bash

    $ rosrun jsk_topic_tools transform_publisher.py /base /right_gripper_vacuum_pad_base -d 2.0

    $ rostopic echo /transform_publisher/output
    header:
      seq: 1
      stamp:
        secs: 1479108976
        nsecs: 799304962
      frame_id: /base
    child_frame_id: /right_gripper_vacuum_pad_base
    transform:
      translation:
        x: 0.614187621295
        y: -0.70974742075
        z: 0.332638443526
      rotation:
        x: 0.0231768305754
        y: 0.764117456951
        z: -0.0773262537191
        w: 0.640006247621
    ---
    header:
      seq: 2
      stamp:
        secs: 1479108978
        nsecs: 799195051
      frame_id: /base
    child_frame_id: /right_gripper_vacuum_pad_base
    transform:
      translation:
        x: 0.61417389684
        y: -0.709704671921
        z: 0.332801975644
      rotation:
        x: 0.0237198976862
        y: 0.764003459868
        z: -0.0768511001916
        w: 0.640179653038
    ---
