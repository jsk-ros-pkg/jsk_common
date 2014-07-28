^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multi_map_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* enable depend to map_server for default
* Contributors: Kei Okada

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

1.0.6 (2014-04-07)
------------------
* Added support for YAML-CPP 0.5+.
  The new yaml-cpp API removes the "node >> outputvar;" operator, and
  it has a new way of loading documents. There's no version hint in the
  library's headers, so I'm getting the version number from pkg-config.
  This is a port of @ktossell's patch for map_server.
* Contributors: Scott K Logan

1.0.5 (2014-03-31)
------------------
* check if map_server exists under /opt/ros/{ROS_DISTRO}/stacks/navigation/map_server for groovy
* Contributors: Kei Okada

1.0.4 (2014-03-27)
------------------
* multi_map_server: disable map_server for default in build_depend, run_depend
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* multi_map_server: catkinize
* Contributors: Kei Okada, Ryohei Ueda

1.0.3 (2014-03-19)
------------------
* update revision number to 1.0.3

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* add explicit dependency to yaml-cpp as yaml-cpp i sinstalled as a rosdep system dependency
* add multi_map_server, map_server with switch service, (this will publish TF between maps in the future)
* Contributors: k-okada, manabu
