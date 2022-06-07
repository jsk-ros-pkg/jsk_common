static_tf_republisher.py
========================


What is this?
-------------


Node to republish /tf_static from a rosbag file


Usage
-----

```
rosrun jsk_topic_tools static_tf_republisher.py _file:=<absolute file path to a rosbag file>
```

or

```
rosrun jsk_topic_tools static_tf_republisher.py <file path to a rosbag file>
```

Sample
------

```
roslaunch jsk_topic_tools sample_static_tf_republisher.launch
```

Parameters
----------

* ``~file`` (String)

  Absolute file path to a rosbag file. Positional argument is prioritized over this parameters.

* ``~mode_static`` (Bool, default: True)

  If set to True, static transforms in a rosbag file are published to ``/tf_static``. Otherwise, they are published to ``/tf``.

* ``~publish_rate`` (Float, default: 10)

  Publishing rate of ``/tf`` if ``~mode_static`` is set to False.
