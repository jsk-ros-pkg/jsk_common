^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2014-03-27)
------------------
* Added missing cmake_minimum_version to CMakeLists
* Contributors: Scott K Logan

1.0.3 (2014-03-19)
------------------
* jsk_tools: update to revision 1.0.3
* jsk_tools: catkinize, add cmake/download_package.cmake

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
* reduce too many ROS_IP and ROS_HOSTNAME printing
* look for address if ROS_IP is blank , see `#41 <https://github.com/jsk-ros-pkg/jsk_common/issues/41>`_
* update getting ip from hostname
* remove ROS_IP and ROS_HOSTNAME if can not find address, see issue `#41 <https://github.com/jsk-ros-pkg/jsk_common/issues/41>`_
* fix for using localhost at ROS_MASTER_URI
* add print_msgs_srvs.sh
* 
* update rossetip using ethernet device or ROS_MASTER_URI
* add mjpeg_server to install ros-%DISTRIBUTION%-mjpeg-server
* add removing of LF on Linux because previous ROS_IP setting does not work machines which has several IP address
* add bashrc.ros
* remove glc and ttf-msconrefonts-install from rosdep due to newer rosdep API w/o bash script
* update manifest for fuerte
* set setlocalmovie==True as defulat
* fixed download links of movies to jenkins
* added url tag for sphinx, all user will get movie from jenkins unless they use -setlocalmovie option
* rename rosdep name for fuerte/rosdep2 : python-docutils -> python-sphinx
* write command output to gtest xml files
* add to check image size
* add debug message
* changed to use codecs.open for utf-8 japanese text
* changed to output mpeg4 video
* changed mjpeg_capture.sh to wait to start listening the port
* update video_directive to show direct link to mp4
* changed node_graph.py, add output /tmp/graph.png, add fill color style
* support --output option
* rewrite & update ogv_encode, generate mp4 and ogv for html5 support
* remove gif support
* rewrite update glc_encode, check video stream and automatically generate for all context
* use theora to convert to ogv to generate theora codec video
* generate webm file for html5
* add video_directive support
* add more message when converting to gif
* update parameters to generate smaller image
* use arista to convert from ogv to mp4
* add arist and recordmydesktop
* add ogv_encode.sh
* remove intermediate files
* update glc_encocde, use compare to check if the glc movie has started or not
* use compare command to skip initial sequence
* add --loop and speedup (delay=10)
* fix option name in src/glc_encode.sh
* update package decision algorithm
* use glfsicle instead of convert to generate animation gif
* add script for colored rxgraph by package
* add dummy ,text in getopt for rostest -t
* fix typo imagemagic -> imagemagick
* add imagemagic python-docutils
* add wkhtmltopdf
* use nextimg to generate gif
* when glc_encode.sh --ctx option is 0, then generate video for each ctx. When generate videos in rostest, rviz run again and overwrite .glc file
* add dependency of jsk_tools to mjpeg_server
* fix to write output file
* added gif maker using glc and convert
* add dummy output
* rename ffmpeg-jsk -> ffmpeg-bin
* added ffmpeg-jsk pkg for avoiding name collision of ffmpeg
* add capture script for mjpeg stream
* gtest_output option is needed, sorry
* add ctx option and output option to glc_encode script
* add glc_encode and rosdep to glc and ffmpeg
* add src/jsk_tools/rosfile_directive.py
* add output_filename
* sort by filename
* add shelblock_rirective from openrave/docs/sphinxext
* add Last Update in HTML
* add description
* doc updates
* doc update
* minor doc changes in jsk_tools
* moved posedetection_msgs, sift processing, and other packages to jsk_common and jsk_perception
* minor doc stuff
* updated jsk_tools url
* more autodoc stuff
* auto-generation of roslaunch docs
* updated launch doc
* updated launch doc
* updated launch doc
* Contributors: chen, k-okada, manabu, nozawa, rosen, y-tnaka, youhei
