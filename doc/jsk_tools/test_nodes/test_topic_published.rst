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
