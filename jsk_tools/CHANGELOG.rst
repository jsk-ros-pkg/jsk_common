^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* [jsk_tools] Add new replace rule to force_to_rename_changelog_user.py
* add error message when percol is not installed
* [jsk_tools] Add percol utility
* update to use rossetmaster in functions
* [jsk_tools] Add progress bar to force_to_rename_changelog_user.py
* [jsk_tools] Add more name conevrsion rule to force_to_rename_changelog_user.py
* [jsk_tools] install bin directory
* Contributors: Ryohei Ueda, Kei Okada

1.0.58 (2015-01-07)
-------------------
* Add more user replacing rules
* Reuse isMasterAlive function across scripts which
  want to check master state
* Add script to change contributors name in CHANGELOG.py
* add roscore_check
* Contributors: Ryohei Ueda, JSK Lab member

1.0.57 (2014-12-23)
-------------------
* add hardware id tp battery capacity
* Contributors: Kei Okada

1.0.56 (2014-12-17)
-------------------
* Use ping with 10 seconds timeout to check master aliveness
* add battery full capacity summary script
* Contributors: Ryohei Ueda, Yuto Inagaki

1.0.55 (2014-12-09)
-------------------
* Add document about roscore_regardless.py
* Check master is reachable before chcking master is alive
* Merge pull request `#613 <https://github.com/jsk-ros-pkg/jsk_common/issues/613>`_ from k-okada/show_ip
  show ROS_IP in prompt
* Merge pull request `#612 <https://github.com/jsk-ros-pkg/jsk_common/issues/612>`_ from k-okada/rename_rossetrobot
  rename rossetrobot -> rossetmaster
* show ROS_IP in prompt
* rename rossetrobot -> rossetmaster, keep rossetrobot for backword compatibility
* add: zshrc.ros (Change emacs mode configuration: Shell-script -> shell-script)
* add: zshrc.ros
* fix prompt when rossetlocal is called.
* Contributors: Ryohei Ueda, Kei Okada, Masaki Murooka, Kentaro Wada

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------
* Ignore exception during kill child process of the process
  launched by roscore_regardless.py
* Contributors: Ryohei Ueda

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------
* add path for android in bashrc.ros
* Contributors: Masaki Murooka

1.0.49 (2014-10-13)
-------------------
* Add script to kill/respawn automatically according to roscore status
* Contributors: Ryohei Ueda

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------
* if user specify ip address by arguments, then we'll use this
* set IP of first candidates
* set /sbin to PATH

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
* add script to doctor workspace
* Contributors: Ryohei Ueda

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

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------
* (ros.bashrc) change PS1 to show current MASTER_URI
* Contributors: Kei Okada

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
* remove depend to mjpeg_server since this is not installed via package.xml
* Contributors: Kei Okada

1.0.4 (2014-03-27)
------------------
* Added missing cmake_minimum_version to CMakeLists
* Contributors: Scott K Logan

1.0.3 (2014-03-19)
------------------
* jsk_tools: update to revision 1.0.3
* jsk_tools: catkinize, add cmake/download_package.cmake

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
* Contributors: nozawa, manabu, Kei Okada, youhei, rosen, chen, y-tnaka
