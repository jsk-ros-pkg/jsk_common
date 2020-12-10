static_transform_republisher.py
===============================


What is this?
-------------


Node to republish /tf_static from a rosbag file


Usage
-----

```
rosrun jsk_topic_tools static_transform_republisher.py _file:=<absolute file path to a rosbag file>
```

or

```
rosrun jsk_topic_tools static_transform_republisher.py <file path to a rosbag file>
```

Parameters
----------

* ``~file`` (String)

  Absolute file path to a rosbag file. Positional argument is prioritized over this parameters.
