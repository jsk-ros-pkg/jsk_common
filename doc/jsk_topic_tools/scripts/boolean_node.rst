boolean_node.py
===============


What is this?
-------------

A node that takes bool values and returns the result of the boolean operation such as ``or``, ``and`` and ``xor``.

For example, this node can be used to merge the results of a node that outputs whether the robot is speaking in English or Japanese.


Subscribing Topics
------------------

Input is prepared for the ``~number_of_input``. A number suffix is added to the ``~input``.
The suffixes start with ``1``. If ``~number_of_input`` equals 2, subscribing topic names are ``~input1`` and ``~input2``.

* ``~input{%d}`` (``std_msgs/Bool``)

  input bool value.


Publishing Topics
-----------------

* ``~output`` (``std_msgs/Bool``)

  The result of the boolean operation.


Parameters
----------

* ``~operator`` (String, required)

  You can choose ``or``, ``and`` and ``xor``.

* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].

* ``~number_of_input`` (Int, Default: ``2``)

  Number of input.


Example
-------

.. code-block:: bash

  roslaunch jsk_topic_tools sample_boolean_node.launch
  rostopic echo /robotsound/is_speaking
  rostopic echo /robotsound_jp/is_speaking
  rostopic echo /is_speaking  # or
  rostopic echo /both_are_speaking  # and
  rostopic echo /either_one_is_speaking  # xor
