^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.72 (2015-06-07)
-------------------
* add network speed check func
* [jsk_tools] update mesage format in sanity_lib's functions
* [jsk_tools/sanity_libs.py] modify small parts of bags
  - use `grep -v grep`
  - remove duplicated if
  - remove unneeded message
* [jsk_tools ] fix type in checkBlack
* [jsk_tools] move ws_doctor.py's function to sanity_lib.py
* [jsk_tools] Add Checker for where rosmaster came from
* [jsk_tools][sanity_lib.py] add bad process checker/killer
* [sanity_lib.py] more message for check silver
* [jsk_tools] add indexMessage func for Index in terminal
* add host option to USB Check
* check local remote ROS Parameter diff
* add expect of usb to check
* add sub ok/error message option to isMasterHostAlive
* add check SilverHammer's published topic hz check
* add timeout 0.001 for rossetip
* [jsk_tools] Add sanity function to check CLOSE_WAIT
* add check usb exist with lsusb
* Add echo option for checkIsTopicPublished
* Add other ros related checker
* [jsk_tools] Support parent workspace in ws_doctor.py
* [jsk_tools] Better output about topics which not working
* [jsk_tools] Use multi-threded sanity lib for faster speed
* [jsk_tools] Fix return value of checkTopicIsPublished
* [jsk_tools] Support multiple topics to check in sanity_lib
* [jsk_tools] Add import math
* [jsk_tools] Add IMU to fix sanity_lib
* [jsk_tools] Add sanity_lib.py for sanity scripts
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Yuto Inagaki, leus

1.0.71 (2015-05-17)
-------------------
* [jsk_tools] Do not run rossetip_addr with device names because it takes
  a lot of time to resolve non-existing host
* [jsk_tools] Allow localhost in check_host_sanity.py
* [jsk_tools/git_commit_aliases] Add 'GitHub' for more easy-to-understand message
* Contributors: Ryohei Ueda

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05)
-------------------
* [jsk_tools] Add -N option to exclude messages from specified nodes
* Contributors: Ryohei Ueda

1.0.68 (2015-05-05)
-------------------

1.0.67 (2015-05-03)
-------------------
* [jsk_tools] return error status when unable ``rossetip``
* Merge remote-tracking branch 'refs/remotes/origin/master' into add-level
  Conflicts:
  jsk_tools/bin/ros_console.py
* [jsk_tools] Add -l option to specify level in ros_console.py
* [jsk_tools] does not support sh but only bash and zsh
* [jsk_tools] store correctly default rosmaster by rossetdefault in bash
  issue: https://github.com/jsk-ros-pkg/jsk_common/issues/899
* [force_to_rename_changelog_user.py] keep order of Contributors
* [force_to_rename_changelog_user.py] add manabu -> Manabu Saito
* Merge pull request `#892 <https://github.com/jsk-ros-pkg/jsk_common/issues/892>`_ from garaemon/add-slash-prefix
  [jsk_tools] Add / prefix to node names in ros_console.py
* [jsk_tools] Add / prefix to node names in ros_console.py
* [jsk_tools] Print more detailed timestamp in ros_console.py
* [jsk_tools] temporary change to avoid error caused by bug in ros/catkin repo
* [jsk_tools] Script to check /etc/hosts sanity
* [jsk_tools] See CATKIN_SHELL to find shell
* [jsk_tools] now you can install pygithub3 by rosdep install
* [jsk_tools] save rosdefault file under ROS_HOME
* [env-hooks/99.jsk_tools.bash] fix typo and wrong -q option for cd
* [jsk_tools] Merge 99.jsk_tools.[bash|zsh] to 99.jsk_tools.sh
* [jsk_tools] Update README for PR `#868 <https://github.com/jsk-ros-pkg/jsk_common/issues/868>`_
* [jsk_tools] Add rossetdefault, rosdefault to bashrc.ros
* [jsk_tools] Add rossetdefault, rosdefault to zshrc.ros
* [jsk_tools] Add Documentation for rossetip,rossetlocal,rossetmaster
* [jsk_tools] Remove no need comment
* [jsk_tools] Display ROS_IP in rossetmaster for zsh
* [jsk_tool] Add script to add git commit aliases like commit-ueda
* [jsk_tools] Remove -a option from zshrc.ros
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, iori

1.0.66 (2015-04-03)
-------------------
* [jsk_tools/zshrc.ros] use env-hooks to store contents of zshrc.ros
* Contributors: Kentaro Wada

1.0.65 (2015-04-02)
-------------------
* [jsk_tools/bashrc.ros] remove android settings from bashrc.ros
* [jsk_tools/bashrc.ros] use env-hooks to store contets of bashrc.ros
* Contributors: Kei Okada

1.0.64 (2015-03-29)
-------------------
* [jsk_tools] check NO_ROS_PROMPT environmental variable when updating
  prompt in order not to change prompt by rossetmaster and rossetip
* [jsk_tools] Add rqt_reconfigure to run_depend
* [jsk_tools] Add new rule to replace handle to name
* [jsk_tools] Fix dependency of jsk_tools
* rename rossetrobot -> rossetmaster, keep rossetrobot for backword compatibility
* Contributors: Ryohei Ueda, Kentaro Wada

1.0.63 (2015-02-19)
-------------------
* need to copy global_bin for devel config too
* [jsk_tools] Install jsk_tools/ros_console.py into global bin directory
* Contributors: Ryohei Ueda, Kei Okada

1.0.62 (2015-02-17)
-------------------
* [jsk_tools] Add script to see rosout in terminal
  Fix syntax
* [jsk_tools] Add more user to rename
* [jsk_tools] Install bin directory to lib directory
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------

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
* Contributors: nozawa, Manabu Saito, Kei Okada, youhei, rosen, Xiangyu Chen, y-tnaka
