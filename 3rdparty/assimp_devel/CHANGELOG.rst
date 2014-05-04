^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package assimp_devel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.18 (2014-05-04)
-------------------
* (#408) fix revision of assimp_git
* fix patch for cmake
* catch up with update of assimp
* Contributors: Kei Okada, Yohei Kakiuchi

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------
* remove export for rosbuild at assimp_devel
* Contributors: YoheiKakiuchi

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------
* add missing build_depend packages
* Contributors: Kei Okada

1.0.12 (2014-04-18)
-------------------
* assimp_devel: fix for buildfirm
* Contributors: Kei Okada

1.0.11 (2014-04-18)
-------------------
* fix for problem when we compile collada_urdf_jsk_patch twice (https://github.com/jsk-ros-pkg/jsk_common/pull/394#issuecomment-40704637)
* Contributors: Kei Okada

1.0.10 (2014-04-17)
-------------------
* moved assim_devel from jsk-ros-pkg/jsk_model_tools
* https://github.com/jsk-ros-pkg/jsk_common/pull/387 was wrong, we should not SKIP_PKG_CONFIG
* fix max cpu to 4 for travis
* change assimp -> assimp_devel to avoid confusion, use pkg-config
* remove duplicated files
* remove export
* udpate catkinmake on assimp_devel
* udpate manifest
* udpate makefile
* add --depth=1 parameter to speed up git clone of assimp
* adding mk
* add bug fixed version
* do not use so many cpus
* copying Makefile for fake dependency
* remove debug message
* does not pollute src directory when catkin make
* fixing catkin cmake of assimp_devel
* fixing catkin cmake of assimp_devel
* catkinized assimp_devel
* add patch for obj exporter
* deviding whole patch to small patches
* devide install library from make install
* update patch for assimp
* update assimp_git.patch
* compiling with git repository
* add eus_assimp for eusing assimp library on EusLisp
* move euscollada,collada_tools,assimp_devl to jsk_model_tools
* Contributors: Kei Okada, Ryohei Ueda, Yohei Kakiuchi

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
* move euscollada,collada_tools,assimp_devl to jsk_model_tools
* makeing symbolic link as /usr/local/lib/libassimp.so.3
* fix unit of collada, for complying with gazebo
* assimp_devel: update version for building
* add assimp_devel package for using latest assimp library
* Contributors: youhei
