test_topic_published.py
=======================

What is this?
-------------

This node is to test the expected published topics while running ``rostest``.


Parameters
----------

- ``~topic_%d`` (``str``)

  The name of topics to test if published.

- ``~timeout_%d`` (``int``)

  Timeout for the topic.

- ``~negative_%d`` (``bool``, Default: ``false``, Optional)

  If it's ``True``, it tests if **not** published.

- ``~import`` (``List[str]``, Default: ``[]``, Optional)

  List of Python modules to import and use in ``~condition_%d`` expression.
  For commodity rospy and numpy modules are imported by default.

- ``~condition_%d`` (``str``, Default: ``None``, Optional)

  Check bool value condition using the given Python expression.
  The Python expression can access any of the Python builtins plus:
  ``topic`` (the topic of the message), ``m`` (the message) and ``t`` (time of message).

  For example, topic is ``std_msgs/String`` and if you want to check whether a sentence is a ``hello``, you can do the following.

  .. code-block:: bash
    condition_0: m.data == 'hello'

  If you want to check the frame id of the header, you can do the following.

  .. code-block:: bash
    condition_0: m.header.frame_id in ['base', 'base_link']

  Note that, use escape sequence when using the following symbols ``<(&lt;)``, ``>(&gt;)``, ``&(&amp;)``, ``'(&apos;)`` and ``"(&quot;)``.

  .. code-block:: bash
    condition_0: m.data &lt; 'spbm'

  The modules given to ``~import`` can also be used.

  .. code-block:: bash
    condition_0: 'numpy.linalg.norm([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]) &lt; 10.0'

Example
-------

.. code-block:: xml

  <test test-name="image_view2_topics"
        name="image_view2_topics"
        pkg="jsk_tools" type="test_topic_published.py">
    <rosparam>
      topic_0: /camera_0/image/screenrectangle_image
      timeout_0: 10
      topic_1: /camera_1/image/screenrectangle_image
      timeout_1: 10
    </rosparam>
  </test>
