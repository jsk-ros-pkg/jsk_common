JSK Common Tools
################

Reboot programs according to roscore aliveness
----------------------------------------------
Please use jsk_tools/roscore_regardless.py
.. code-block::

  rosrun jsk_tools roscore_regardless.py your-awesome-program

For example,

.. code-block::
  rosrun jsk_tools roscore_regardless.py roslaunch robot_pkg robot.launch

Auto roslaunch documentation
----------------------------

In order to enable the launch doc generator, have to add the following to each ROS package:

1. a **rosdoc.yaml** file including:

.. code-block:: yaml

  - builder: sphinx
    name: roslaunch scripts

2. in **manifest.xml**:

.. code-block:: xml

  <export>
    <rosdoc config="rosdoc.yaml" />
  </export>

3. in **CMakeLists.txt**:

.. code-block:: cmake

  rosbuild_find_ros_package("jsk_tools")
  execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)

4. Because of current ROS limitations, make sure to add the generated files inside your local repository! It seems that the ros.org servers cannot run the **-builder: rosmake** command to generate this file:

.. code-block:: bash

  svn add index.rst conf.py
