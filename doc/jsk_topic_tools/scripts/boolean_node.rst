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


  Note that this condition is evaluated each time a topic is published. For example, a condition that checks whether a certain topic has arrived within one second look like this.

  .. code-block:: bash

    input1_condition: "(rospy.Time.now() - t).to_sec() &lt; 1.0"

  Use escape sequence when using the following symbols <(``&lt;``), >(``&gt;``), &(``&amp;``), '(``&apos;``) and "(``&quot;``) in launch file.


* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].

* ``~number_of_input`` (Int, Default: ``2``)

  Number of input. ``~number_of_input`` should be greater than 0.


Example
-------

.. code-block:: bash

  $ roslaunch jsk_topic_tools sample_boolean_node.launch


The outputs of a simple Boolean operation are as follows.


.. code-block:: bash

  $ rostopic echo /robotsound/is_speaking -n1
  data: True
  ---
  $ rostopic echo /robotsound_jp/is_speaking -n1
  data: False
  ---
  $ rostopic echo /is_speaking -n1  # or
  data: True
  ---
  $ rostopic echo /both_are_speaking -n1  # and
  data: False
  ---
  $ rostopic echo /either_one_is_speaking -n1  # xor
  data: True
  ---


In ``sample_boolean_node.launch``, there is a description that gives ``input_condition`` as follows.


.. code-block:: XML

  <node name="boolean_node_checking_conditions"
        pkg="jsk_topic_tools" type="boolean_node.py"
        clear_params="true" >
    <remap from="~input1" to="/image1" />
    <remap from="~input2" to="/image2" />
    <remap from="~input3" to="/chatter" />
    <rosparam>
      number_of_input: 3
      input1_condition: "'base' in m.header.frame_id"
      input2_condition: "'base' in m.header.frame_id"
      input3_condition: m.data == 'hello'
    </rosparam>
  </node>


The output results when using the condition are as follows.


.. code-block:: bash

  $ rostopic echo /image1 -n1
  header:
    seq: 15029
    stamp:
      secs: 0
      nsecs:         0
    frame_id: "base"
  height: 0
  width: 0
  encoding: ''
  is_bigendian: 0
  step: 0
  data: []
  ---
  $ rostopic echo /image2 -n1
  header:
    seq: 32445
    stamp:
      secs: 0
      nsecs:         0
    frame_id: "base_link"
  height: 0
  width: 0
  encoding: ''
  is_bigendian: 0
  step: 0
  data: []
  ---
  $ rostopic echo /chatter -n1
  data: "hello"
  ---
  $ rostopic echo /boolean_node_checking_conditions/output/and -n1
  data: True
  ---
  $ rostopic echo /boolean_node_checking_conditions/output/not -n1
  data: False
  ---
  $ rostopic echo /boolean_node_checking_conditions/output/or -n1
  data: True
  ---
  $ rostopic echo /boolean_node_checking_conditions/output/xor -n1
  data: True
  ---
