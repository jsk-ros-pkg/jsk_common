^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opt_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2014-03-27)
------------------
* opt_camera: include unistd.h on the top
* Added missing unistd.h includes
* Contributors: Kei Okada, Scott K Logan

1.0.3 (2014-03-19)
------------------
* jsk_common: update revision number to 1.0.3
* opt_camera: catkinize

1.0.2 (2014-03-12)
------------------
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: revert opt_camera dependency by using depend tag
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: use rosdep rather than depend
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: remove opt_camra's depend tag to image_proc, because it is a runtime dependency
* `#299 <https://github.com/jsk-ros-pkg/jsk_common/issues/299>`_: add image_proc to opt_camera rosdep
* Contributors: Ryohei Ueda

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* mv package.xml bak.package.xml to avoid documentation error
* add cv_bridge to *_depend
* add package.xml for catkin
* rename
* use cv_bridge because CvBridge deprecated
* fix for fuerte, support ROSPACK_API_V2(fuerte)
* remove stereo_synchronizer from manifest.xml
* change grab code from opencv to v4l2 to support non VGA mode
* change jsk_stereo_proc to stereo_synchronizer
* add -fPIC for relocatable objects
* opt_nm30_viewer now support arguments to set camera_index number
* remove dependency on logitec_pantilt
* change to use query{Omni/Wide/Middle/Narrow}Frame
* add query{Omni/Wide/Middle/Narrow}Frame
* fix name_sapce : opt_cam to camera
* display firmwareVersion and serialId in opt_nm33_camera program
* add opt_camera package for NM33 camera
* Contributors: k-okada, youhei
