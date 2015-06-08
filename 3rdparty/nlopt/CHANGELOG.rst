^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nlopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.72 (2015-06-07)
-------------------

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------

1.0.67 (2015-05-03)
-------------------

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------

1.0.63 (2015-02-19)
-------------------
* [nlopt] Fix nlopt compilation and instlation
* Contributors: Ryohei Ueda

1.0.62 (2015-02-17)
-------------------
* corrected install locations
* flipped lib and include on install directive
* include files weren't properly included and the pattern matching for nlopt libraries needed to be fixed
* Contributors: C. Barrett Ames

1.0.61 (2015-02-11)
-------------------
* install devel files into catkin package destination
* remove install functions,
* change build current directroy to catkin/build, and install lib files to devel/lib
* change build current directroy to catkin/build, and install lib files to devel/lib
* [nlopt] Fix nlopt compilation to export nlopt_cxx correctly.
  * compile libraries under source directory and after that copy to devel space.
  * Use add_custom_command instead of execute_process
* added nlopt_cxx library to catkin_package so that the library will be included by packages that depend on nlopt
* Contributors: Ryohei Ueda, Shintaro Noda, Barrett

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [nlopt] Add LIBS='-stdc++' to avoid link error on saucy
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
* Do not use rospack to build nlopt on catkin
* Contributors: Ryohei Ueda

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------
* Add rospack to nlopt dependency
* Contributors: Ryohei Ueda

1.0.41 (2014-09-23)
-------------------
* Install binaries for deb package
* Contributors: Ryohei Ueda

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
* add catkin.cmake and catkin_package declearation for generating config.cmake
* use PROJECT_SOURCE_DIR value in CMakeLists.txt for Makefile DESTDIR value instead of /home/s-noda/ros/hydro/src/jsk-ros-pkg/jsk_common/3rdparty/nlopt
* fix minor change for amenda
* change output dir from catkin_home -> nlopt dir
* remove rosmake function from CMakeLists.txt
* miss project name fix, nlopt
* add CMakeList and package.xml for catkinize
* Contributors: Shintaro Noda

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

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19 23:29)
-------------------------

1.0.15 (2014-04-19 20:19)
-------------------------

1.0.14 (2014-04-19 12:52)
-------------------------

1.0.13 (2014-04-19 11:06)
-------------------------

1.0.12 (2014-04-18 16:58)
-------------------------

1.0.11 (2014-04-18 08:18)
-------------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------

1.0.6 (2014-04-07)
------------------

1.0.5 (2014-03-31)
------------------

1.0.4 (2014-03-29)
------------------

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* initial commit, nlopt add
* Contributors: Shintaro Noda
