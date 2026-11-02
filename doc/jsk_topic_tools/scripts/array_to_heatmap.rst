array_to_heatmap.py
===================

ROS 1 / ROS 2 node converting a scalar matrix to a ``sensor_msgs/Image`` (``bgr8``).
Each input cell becomes a colored square, with its numeric value shown by default.
The first row is at the top and the first column is at the left.
Cells are not interpolated.

Usage::

  rosrun jsk_topic_tools array_to_heatmap.py __name:=pressure_heatmap \
    ~input:=/sensor/pressure ~output:=/sensor/pressure/heatmap

ROS 2 usage::

  ros2 run jsk_topic_tools array_to_heatmap.py --ros-args \
    -r ~/input:=/sensor/pressure -r ~/output:=/sensor/pressure/heatmap

ROS 2 discovers the input type asynchronously and accepts both best-effort and reliable publishers using sensor-data QoS.
Its output uses reliable QoS.
Parameters are supplied with ``--ros-args -p name:=value`` (without ``~``).
Use ``-p input_type:=sensor_msgs/msg/Image`` to bypass type discovery if desired.
ROS 1 also detects the input type automatically and accepts ``~input_type``.

Input ``~input`` automatically accepts numeric ``std_msgs/*MultiArray`` types (Float32/64, Int8/16/32/64, UInt8/16/32/64), or single-channel ``sensor_msgs/Image``: mono8, mono16, 8UC1, 8SC1, 16UC1, 16SC1, 32SC1, 32FC1 and 64FC1.
MultiArray dimensions must be [rows, columns]; data offsets and row padding are respected.
Image byte order and row padding are respected.
RGB images and arrays with dimensions other than two are rejected.
For an empty MultiArray layout, set ``~rows`` and ``~cols`` explicitly.

Parameters
----------

* ``~input_type`` (empty): auto-discover, or specify ``pkg/Type`` / ``pkg/msg/Type``.
* ``~cell_size`` (64): square cell size in pixels, at least 16.
* ``~show_values`` (true): overlay each cell's original numeric value.
* ``~precision`` (2): number of decimal places, from 0 to 10.
  Very long values use scientific notation so the text fits inside the cell.
* ``~colormap`` (jet): OpenCV colormap name, e.g. jet, hot, viridis.
* ``~min_value``, ``~max_value`` (unset): color scale endpoints.
  Unset endpoints use the current frame's finite extrema.
  Set both for comparable colors across time; values outside the range are clipped for coloring only.
  A constant frame without fixed bounds uses the lowest color.
  NaN and infinity use gray cells and retain their textual labels.
* ``~rows``, ``~cols`` (0): fallback dimensions for arrays without layout.
* ``~frame_id`` (empty): frame for MultiArray input.

Image input retains its original header.
MultiArray input uses the callback arrival time and ``~frame_id``.
Invalid input is skipped with a throttled warning.

Offline conversion
------------------

Use ``jsk_rosbag_tools/add_heatmap.py`` to add a heatmap topic to a new bag while preserving the original messages and their recording times.
See the jsk_rosbag_tools README for CLI examples.
