boolean_node.py
===============


What is this?
-------------

A node that takes bool values and returns the results of the boolean operation such as ``or``, ``and``, ``not`` and ``xor``.

For example, this node can be used to merge the results of a node that outputs whether the robot is speaking in English or Japanese.


Subscribing Topics
------------------

Input is prepared for the ``~number_of_input``. A number suffix is added to the ``~input``.
The suffixes start with ``1``. If ``~number_of_input`` equals 2, subscribing topic names are ``~input1`` and ``~input2``.
In the case of ``not`` operation, only ``~input1`` is subscribed.

* ``~input{%d}`` (``AnyMsg``)

  input value.


Publishing Topics
-----------------

* ``~output/or`` (``std_msgs/Bool``)

  The result of the ``or`` operation.

* ``~output/and`` (``std_msgs/Bool``)

  The result of the ``and`` operation.

* ``~output/not`` (``std_msgs/Bool``)

  The result of the ``not`` operation.

* ``~output/xor`` (``std_msgs/Bool``)

  The result of the ``xor`` operation.


Parameters
----------


* ``~input{%d}_condition`` (String, Default: ``m.data``)

  Returning bool value condition using the given Python expression.
  The Python expression can access any of the Python builtins plus:
  ``topic`` (the topic of the message), ``m`` (the message) and ``t`` (time of message).

  For example, ``~input1`` topic is ``std_msgs/String`` and if you want to check whether a sentence is a ``hello``, you can do the following.

  .. code-block:: bash

    input1_condition: m.data == 'hello'


  If you want to check the frame id of the header, you can do the following.

  .. code-block:: bash

    input1_condition: m.header.frame_id in ['base', 'base_link']


* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].

* ``~number_of_input`` (Int, Default: ``2``)

  Number of input. ``~number_of_input`` should be greater than 0.


Example
-------

.. code-block:: bash

  roslaunch jsk_topic_tools sample_boolean_node.launch
  rostopic echo /robotsound/is_speaking
  rostopic echo /robotsound_jp/is_speaking
  rostopic echo /is_speaking  # or
  rostopic echo /both_are_speaking  # and
  rostopic echo /either_one_is_speaking  # xor
  rostopic echo /robot_is_not_speaking  # not
