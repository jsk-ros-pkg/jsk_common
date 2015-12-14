test_stdout.py
==============

What is this?
-------------

This node is to test the expected stdout of shell command while running ``rostest``.


Parameters
----------

- ``~command`` (``str``)

  The command to run.

- ``~shell`` (``bool``)

  Run the command with shell mode or not.
  See `here <https://docs.python.org/2/library/subprocess.html#using-the-subprocess-module>`_ for more detail.

- ``~stdout`` (``str``)

  Expected stdout.

- ``~stdout_line%d`` (``str``)

  Expected stdout of the specific line.


Example
-------

.. code-block:: xml

  <test test-name="test_stdout"
        pkg="jsk_tools" type="test_stdout.py">
    <param name="command" value="timeout 10 rostopic echo /label_image_decomposer/output/header/frame_id -n1 || true" />
    <param name="stdout_line0" value="camera" />
    <param name="shell" value="true" />
  </test>
