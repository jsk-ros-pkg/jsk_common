Auto roslaunch documentation
----------------------------

In order to enable the launch doc generator, have to add the following to each ROS package:

1. a **rosdoc.yaml** file including:

.. code-block:: yaml

  - builder: rosmake
  - builder: sphinx
    output_dir: launch
    name: roslaunch scripts

2. in **manifest.xml**:

.. code-block:: xml

  <export>
    <rosdoc config="rosdoc.yaml" />
  </export>

3. in **CMakeLists.txt**:

.. code-block:: cmake

  rosbuild_find_ros_package("jsk_tools")
  execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} ./${jsk_tools_PACKAGE_PATH}/launchdoc-generator.py ${PROJECT_NAME} --output_dir=. --nomakefile RESULT_VARIABLE _make_failed)
