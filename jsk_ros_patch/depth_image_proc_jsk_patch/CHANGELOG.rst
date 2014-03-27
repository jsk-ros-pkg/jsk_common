^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depth_image_proc_jsk_patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* jsk_common: update revision number to 1.0.3
* depth_image_proc_jsk_patch: catkinize (dummy)
* Contributors: Kei Okada

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* create new ticket for trac
* modify patch. we now don't need openni_launch_jsk_patch
* modify trac link
* move sample_zoom.launch from depth_image_proc_jsk_patch to openni_launch_jsk_patch and update sample calibration data
* update undistort.cpp for the change of kinect_near_mode_calibration/src/calibrate.cpp
* modify the repository of depth_image_proc. it is for fuerte. and remove openni_launch patch. this patch will be commited as an another package
* use http instead of https to avoid certificate verify failure
* modify undistort.cpp according to the change of kinect_near_mode_calibration. fitting with depth information
* miscommit wrong parameter file by mistake
* add depth_image_proc_jsk_patch. this package is a patch to undistort pointcloud acquired from kinect with zoom lens
* Contributors: k-okada, tsuda
