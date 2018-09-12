test_node_alive.py
==================

What is this?
-------------

This node tests if the expected nodes are alive while running ``rostest``.


Parameters
----------

- ``~node_%d`` (``str``)

  The name of nodes to test if alive.

- ``~timeout_%d`` (``int``)

  Timeout for the node.

- ``~negative_%d`` (``bool``, Default: ``false``, Optional)

  If this parameter is set to ``True``, this node tests if the expected node is **not** alive.


Example
-------

.. code-block:: xml

  <test test-name="camera_nodes"
        name="camera_nodes"
        pkg="jsk_tools" type="test_node_alive.py">
    <rosparam>
      node_0: /camera_0/camera_nodelet_manager
      timeout_0: 10
      node_1: /camera_1/camera_nodelet_manager
      timeout_1: 10
    </rosparam>
  </test>
