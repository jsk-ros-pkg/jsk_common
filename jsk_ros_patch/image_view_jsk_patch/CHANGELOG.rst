^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_view_jsk_patch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.0.4 (2014-03-27)
------------------
* image_view_jsk_patch: catkinize (dummy)

1.0.0 (2014-03-05)
------------------
* added code to check and make sure there is image data when the user inputs command to save image
* dir_name cannot be specified using relative path since ros launches node in different directory. Added in print out message to notify user about this when file saving fails.
* added in check to see if cv::imwrite returned true before printing out message to say image save succeeded
* modify to use revision image_pipeline-1.8.4 (fuerte tags). it works also in electric
* change to use git repository instead of svn. it is preparation for fuerte and groovy version
* add new patch to update parameter anytime callback is called. and copy image_saver binary file to bin directory to be able to execute
* fix SVN_URL path in Makefile
* add image_saver-encoding
* clean up patches
* add trac number to Makefile
* add image_view_jsk_patch. this is a patch of image_saver.cpp in image_view. you can call service to save subscribed image. also you can set saving image directory and image encodings
* Contributors: k-okada, tsuda, wesley
