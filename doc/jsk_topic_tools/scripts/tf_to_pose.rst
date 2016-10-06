tf_to_pose.py
=============


What is this?
-------------

Tool to convert `/tf` to `geometry_msgs::PoseStamped`.


Subscribing Topics
------------------

- `/tf` (`tf2_msgs/TFMessage`)

  Transformation.


Parameters
----------

- `src_frame`, `dst_frame` (String, required)

  Frames to compute transform to get the relative pose.

- `rate` (Float, default: 1.)

  Publishing rate [Hz].


Example
-------

.. code-block:: bash

  rosrun tf static_tf_transform 0 0 1 0 0 0 base_link head_link 100
  rosrun jsk_topic_tools tf_to_pose.py _src_frame:=base_link _dst_frame:=head_link
  rostopic echo /tf_to_pose/output
