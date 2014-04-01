^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamic_tf_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2014-03-27)
------------------
* dynamic_tf_publisher: add rospy to depends

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------
* fix maintainer name

1.0.0 (2014-03-05)
------------------
* set all package to 1.0.0
* catkinize dynamic_tf_publisher
* revert commit rev 5550
* set use cache false by default
* add parameter to select whether to use cache or not
* fix the bug in dynamic_tf_publisher package
* see ROS_DISTRO to use genpy.message or roslib.message (old API)
* save tf-chain in rosparm, in case of when tf_publisher is respawned
* roslib/Header is old style
* debug delete callback to work /delete_tf service
* publish tfMessage to ~tf, because it will ease debugging,
  and add some debug print in assoc callback
* DissocTFRequest does not have child_frame, it has frame_id slot
* add delete tf service
* fix error check of assocTF
* fix bag when assoc service called again
* do not accept set_dynamic_tf service for assocd frames
* mv jtalk and pddl to 3rdparty directory
* Contributors: Kei Okada, furuta, k-okada, kazuto, manabu, ueda
