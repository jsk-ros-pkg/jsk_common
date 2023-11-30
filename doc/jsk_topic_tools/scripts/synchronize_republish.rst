synchronize_republish.py
========================


What is this?
-------------


Node to republish multiple topics with exact/approximate synchronization.


Subscribing Topics
------------------

This is assigned by ``~topics`` rosparam.


Publishing Topics
-----------------

* ``~pub_{0>2}``

  ``{0>2}`` represents the index of the topic assigned by ``~topics`` rosparam,
  for example ``~pub_00``, ``~pub_11``.


Parameters
----------

* ``~topics`` (List of string, Required)

  Topics to synchronize.

* ``~approximate_sync`` (Bool, Default: ``false``)

  If ``true``, approximate synchronization is used for synchoronizing assigned topics.

* ``~allow_headerless`` (Bool, Default: ``false``)

  If ``true``, ``allow_headerless`` option of approximate synchronization is enabled.

* ``~queue_size`` (Integer, Default: ``100``)

  Queue length for subscribing topics.

* ``~slop`` (Double, Default: ``0.1``)

  Allowable misalignment time between the subscribing topics.
