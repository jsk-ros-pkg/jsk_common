and_node.py
===========


What is this?
-------------

A node that takes bool values and returns the result of the ``and`` operation.


Subscribing Topics
------------------

* ``~input01`` (``std_msgs/Bool``)

  input bool value.

* ``~input02`` (``std_msgs/Bool``)

  input bool value.


Publishing Topics
-----------------

* ``~output`` (``std_msgs/Bool``)

  The result of the ``and`` operation.


Parameters
----------

* ``~rate`` (Int, Default: ``100``)

  Publishing rate [Hz].
