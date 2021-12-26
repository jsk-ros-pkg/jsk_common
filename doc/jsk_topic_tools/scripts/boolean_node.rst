boolean_node.py
===============


What is this?
-------------

A node that takes bool values and returns the result of the boolean operation such as ``or``, ``and`` and ``xor``.

For example, this node can be used to merge the results of a node that outputs whether the robot is speaking in English or Japanese.


Subscribing Topics
------------------

Input is prepared for the ``~number_of_input``. A number suffix is added to the ``~input``.

* ``~input{%d}`` (``std_msgs/Bool``)

  input bool value.


Publishing Topics
-----------------

* ``~output`` (``std_msgs/Bool``)

  The result of the boolean operation.


Parameters
----------

* ``~operation`` (String, required)

  You can choose ``or``, ``and`` and ``xor``.

* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].

* ``~number_of_input`` (Int, Default: ``2``)

  Number of input.
