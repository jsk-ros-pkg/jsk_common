^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [rosping] Do not test on indigo, it seems to be broken
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------

1.0.36 (2014-09-01)
-------------------

1.0.35 (2014-08-16)
-------------------

1.0.34 (2014-08-14)
-------------------

1.0.33 (2014-07-28)
-------------------

1.0.32 (2014-07-26)
-------------------

1.0.31 (2014-07-23)
-------------------

1.0.30 (2014-07-15)
-------------------

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------

1.0.21 (2014-05-20)
-------------------

1.0.20 (2014-05-09)
-------------------
* rosping: setuid via install()
* Contributors: Kei Okada

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------
* (rosping) Add pkg description, clarify difference with simular tool.
* Contributors: Isaac IY Saito

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

1.0.8 (2014-04-11)
------------------

1.0.3 (2014-03-19)
------------------
* display how to set uid for rosping
* use DISDIR in install(CODE) to make rosping work
* Contributors: Kei Okada

1.0.1 (2014-03-07)
------------------
* Merge pull request `#293 <https://github.com/jsk-ros-pkg/jsk_common/issues/293>`_ from k-okada/sudo_rosping
  rosping :  add install and test
* keep persmissions during installation
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* add rosdep name
* rosping: install test directory
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* add rosdep name
* keep persmissions during installation
* rosping: install test directory
* Merge pull request `#283 <https://github.com/jsk-ros-pkg/jsk_common/issues/283>`_ from k-okada/release

1.0.0 (2014-03-05)
------------------
* Merge pull request `#283 <https://github.com/jsk-ros-pkg/jsk_common/issues/283>`_ from k-okada/release
  add meta package and set all package.xml to 1.0.0
* set all package to 1.0.0
* add rostest to rosping
* (rosping/catkin.cmake) try sudo see if it works
  add -n option, not to ask password
* install rosping
* fixing ping timing
* chack the arguments after ros::init
* adding ~rate parameter
* fix: display how to setuid at the end of cmake
* display how to setuid at the end of cmkae
* fix : make catkin to work rosping
* fix catkin make
* publish -1 when connection timed out
* catkinize rosping
* add rosping
* Contributors: Ryohei Ueda, Kei Okada, Yusuke Furuta
