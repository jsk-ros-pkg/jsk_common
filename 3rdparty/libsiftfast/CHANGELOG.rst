^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libsiftfast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------
* change libsiftfast to non-catkin package by add SKIP_CMAKE_CONFIG_GENERATION
* Contributors: Kei Okada

1.0.8 (2014-04-11)
------------------
* Merge pull request #376 from k-okada/catkinize_lib_siftfast
* fix for buildpakcage: use install(CODE for libraries, since library file is generated during compile phase; remove devel directory when dhbuild; install share/siftfast -> share/libsiftfast
* Contributors: Kei Okada
* Only run Makefile during build phase (not install)
  Currently, `Makefile` is re-run when catkin installs the package. This causes `Makefile` to re-install, this time leaving the files in `/` instead of an intermediate directory. This ensures that once built, `Makefile` is not re-run.
* Contributors: Scott K Logan

1.0.7 (2014-04-10)
------------------
* Added missing build_depend on rospack and roslib
* Handle case where ROS_DISTRO is not set
* Contributors: Scott K Logan

1.0.6 (2014-04-07)
------------------
* catkinize libsiftfast, add fake add_library, set_target_properties for catkin, groovy does not suport EXPORTED_TARGETS
* Contributors: Kei Okada

1.0.0 (2014-03-05)
------------------
* add clean patched
* change SVN repository to new sourceforge server. Fixed https://code.google.com/p/rtm-ros-robotics/issues/detail?id=84
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* Contributors: furuta, k-okada, rosen
