not_node.py
===========


What is this?
-------------

A node that takes a bool value and returns the result of the ``not`` operation.


Subscribing Topics
------------------

* ``~input`` (``std_msgs/Bool``)

  input bool value.


Publishing Topics
-----------------

* ``~output`` (``std_msgs/Bool``)

  The result of the ``not`` operation.


Parameters
----------

* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].


Example
-------

.. code-block:: bash

  roslaunch jsk_topic_tools sample_not_node.launch
  rostopic echo /robotsound/is_speaking
  rostopic echo /robot_is_not_speaking  # not
