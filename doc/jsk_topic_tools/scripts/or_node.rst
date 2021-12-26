or_node.py
==========


What is this?
-------------

A node that takes bool values and returns the result of the ``or`` operation.

For example, this node can be used to merge the results of a node that outputs whether the robot is speaking in English or Japanese.


Subscribing Topics
------------------

* ``~input01`` (``std_msgs/Bool``)

  input bool value.

* ``~input02`` (``std_msgs/Bool``)

  input bool value.


Publishing Topics
-----------------

* ``~output`` (``std_msgs/Bool``)

  The result of the ``or`` operation.


Parameters
----------

* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].
