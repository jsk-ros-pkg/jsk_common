^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.11 (2020-07-10)
-------------------
* Fix for noetic build (`#1648 <https://github.com/jsk-ros-pkg/jsk_common/issues/1648>`_)

  * fix for python3, except, print ....
  * jsk_tools: fix for python3
  * pytho3 dislike \d in regrex, src/test_topic_published.py:50:37: W605 invalid escape sequence '\d'
  * python3 need to use a in dict, instead of dict.has_key(a)
  * fix print(), Exception as e for python3
  * migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration
  * upgrade package.xml to format=3

* fix failure on finally clause (`#1645 <https://github.com/jsk-ros-pkg/jsk_common/issues/1645>`_)
* Add debian in generate_deb_status_table.pyf( `#1633 <https://github.com/jsk-ros-pkg/jsk_common/issues/1633>`_)
* write comment on how to generate deb status section. (`#1631 <https://github.com/jsk-ros-pkg/jsk_common/issues/1631>`_)

  * remove --rosdistro-from and --rosdistro-to, get current active rosdistro list from index file
  * write comment on how to generate deb status section.  https://stackoverflow.com/questions/4823468/comments-in-markdown

* [jsk_tools] Add --ping-trials option to roscore_regardless.py (`#1632 <https://github.com/jsk-ros-pkg/jsk_common/issues/1632>`_)

  * Sometimes ping is not stable. `--ping-trials` option enables roscore_regardless.py to verify host computer of rosmaster is alive by multi-times ping commands.

* [jsk_tools] Show voltage in battery_capacity_summary.py (`#1628 <https://github.com/jsk-ros-pkg/jsk_common/issues/1628>`_)

  * Fix output format in battery_capacity_summary.py
  * Show voltage as well in battery_capacity_summary.py
  * Show unit of full/remaining capacity in battery_capacity_summary.py

* [jsk_tools/roscore_regardless.py] Do not send SIGTERM before roslaunch sends SIGTERMf( `#1627 <https://github.com/jsk-ros-pkg/jsk_common/issues/1627>`_)

  * Add option to change timeout duration to escalate signals

* [jsk_tools] Add --timeout option to roscore_regardless.py ( `#1622 <https://github.com/jsk-ros-pkg/jsk_common/issues/1622>`_)

  * Add --timeout option to change timeout duration of ping command towards rosmaster computer.
  * --timeout option defaults to 10 seconds.

* battery_capacity_summary.py: fix order of columns (`#1619 <https://github.com/jsk-ros-pkg/jsk_common/issues/1619>`_)
* Contributors: Yuki Furuta, Kei Okada, Ryohei Ueda, Shingo Kitagawa, Yuto Uchimi

2.2.10 (2018-11-03)
-------------------

2.2.9 (2018-11-02)
------------------

2.2.8 (2018-11-01)
------------------
* Add comment for install destination (`#1604 <https://github.com/jsk-ros-pkg/jsk_common/issues/1604>`_)
* jsk_tools/src/jsk_tools/bag_plotter.py: Correct a line mistaken, easy but critical (`#1602 <https://github.com/jsk-ros-pkg/jsk_common/issues/1602>`_)
* jsk_tools/src/jsk_tools/bag_plotter.py: Add style option for line color and font size (`#1601 <https://github.com/jsk-ros-pkg/jsk_common/issues/1601>`_)
* [jsk_tools] add python-progressbar to run_depend (`#1588 <https://github.com/jsk-ros-pkg/jsk_common/issues/1588>`_)
* Contributors: Kei Okada, Ryosuke Tajima, Yuto Uchimi

2.2.7 (2018-06-27)
------------------
* add missing dirs into install command (`#1583 <https://github.com/jsk-ros-pkg/jsk_common/issues/1583>`_)
  * add missing dirs into install command
* Fix roscore regardless (`#1576 <https://github.com/jsk-ros-pkg/jsk_common/issues/1576>`_)
  * respawn child process if -r is given
  * jsk_tools: roscore_regardless.py: fix to work at more cases
* Contributors: Yuki Furuta, Yasuhiro Ishiguro

2.2.6 (2018-01-05)
------------------
* [jsk_tools] add ROS param set test (`#1535 <https://github.com/jsk-ros-pkg/jsk_common/issues/1535>`_)
* jsk_tools: fix sanity lib test (`#1573 <https://github.com/jsk-ros-pkg/jsk_common/issues/1573>`_)
* update generate_deb_status_table.py (`#1539 <https://github.com/jsk-ros-pkg/jsk_common/issues/1539>`_)
  * default rosdistro-to is lunar
  * use python-rosdistro to create DISTRS Dict
* Support network interface name convention from ubuntu 15.10 (`#1561 <https://github.com/jsk-ros-pkg/jsk_common/issues/1561>`_)
  * Fixes https://github.com/jsk-ros-pkg/jsk_common/issues/1559
    c.f.: https://askubuntu.com/questions/702161/why-is-my-interface-now-wlp2s0-instead-of-wlan0
* Make TopicPublishedChecker run in parallel (`#1546 <https://github.com/jsk-ros-pkg/jsk_common/issues/1546>`_)
  * Make TopicPublishedChecker multi-processable
* Fix undefined variable arg in rosview on bash (`#1545 <https://github.com/jsk-ros-pkg/jsk_common/issues/1545>`_)
* Contributors: Kentaro Wada, Shingo Kitagawa, Yuki Furuta

2.2.5 (2017-06-19)
------------------

2.2.4 (2017-06-14)
------------------
* CMakeLists.txt : fix install process, bin is already installed to /opt/ros/indigo/lib/jsk_tools/ directory (`#1518 <https://github.com/jsk-ros-pkg/jsk_common/issues/1518>`_)
* Remove import error message in ros_console.py, Because python-colorama is installed via apt (`#1517 <https://github.com/jsk-ros-pkg/jsk_common/issues/1517>`_)
* Contributors: Kei Okada, Kentaro Wada

2.2.3 (2017-03-23)
------------------
* jsk_tools/src/jsk_tools/migration.py: Fix migration of from XXX.[srv|msg] import YYY (`#1506 <https://github.com/jsk-ros-pkg/jsk_common/issues/1506>`_)
* jsk_tools/src/generate_deb_status_table.py: Add python-tabulate-pip as a dependency (`#1505 <https://github.com/jsk-ros-pkg/jsk_common/issues/1505>`_)
* [jsk_tools] Improve test_topic_published.py (check /use_sim_time neatly) (`#1504 <https://github.com/jsk-ros-pkg/jsk_common/issues/1504>`_)
  * jsk_tools/src/test_topic_published.py: Simplify negative check in test_topic_published.py
  * Check /clock publication neatly and fails if timed out
  * Use PublishChecker merge in ros/ros_comm
* jsk_tools/src/generate_deb_status_table.py: Cope with arm64 for deb status table (`#1503 <https://github.com/jsk-ros-pkg/jsk_common/issues/1503>`_)
* Contributors: Kentaro Wada

2.2.2 (2016-12-30)
------------------
* package.xml : rosemacs-el is only available until precise, from indigo, we uses rosemacs (`#1497 <https://github.com/jsk-ros-pkg/jsk_common/issues/1497>`_)
* src/rostopic_host_sanity : Check host sanity with a script
* Contributors: Kei Okada, Kentaro Wada

2.2.1 (2016-12-13)
------------------
* env-hooks/99.jsk_tools.sh: Set WITHOUT_ROS_PROMPT at _update_prompt to fix `#1494 <https://github.com/jsk-ros-pkg/jsk_common/issues/1494>`_
* src/generate_deb_status_table.py : Cope with xenial + arm for deb status table (`#1491 <https://github.com/jsk-ros-pkg/jsk_common/issues/1491>`_)
* src/generate_deb_status_table.py : Support arm build in
  generate_deb_status_table.py : Generate deb release table with python script (`#1490 <https://github.com/jsk-ros-pkg/jsk_common/issues/1490>`_)
* [jsk_tools][99.jsk_tools.sh] fix: issue `#1472 <https://github.com/jsk-ros-pkg/jsk_common/issues/1472>`_
* Contributors: Kentaro Wada, Yuki Furuta

2.2.0 (2016-10-28)
------------------
* jsk_tools/src/post_to_slack_server: Post to slack via String message input (`#1466 <https://github.com/jsk-ros-pkg/jsk_common/issues/1466>`_)
* jsk_tools/src/jsk_tools/migration.py: Add utility class to migrate rosmsg (`#1464 <https://github.com/jsk-ros-pkg/jsk_common/issues/1464>`_)
* jsk_tools/src/jsk_tools/sanity_lib.py : Support echo_noarr in
  checkTopicIsPublished (`#1459 <https://github.com/jsk-ros-pkg/jsk_common/issues/1459>`_)
* Contributors: Kentaro Wada

2.1.2 (2016-09-14)
------------------

2.1.1 (2016-09-07)
------------------

2.1.0 (2016-09-06)
------------------
* now wstool info can run from any directory (`#1452 <https://github.com/jsk-ros-pkg/jsk_common/issues/1452>`_)
  Closes (`#1317 <https://github.com/jsk-ros-pkg/jsk_common/issues/1318>`_)

* use ip command for rossetip for better compatibility (`#1436 <https://github.com/jsk-ros-pkg/jsk_common/issues/1436>`_)

  * [jsk_tools] add iproute2 to run_depend
  * [jsk_tools/env-hooks/99.jsk_tools.sh] use ip for rossetip instead of ifconfig

* Stamped filename for recoding video with axis camera (`#1424 <https://github.com/jsk-ros-pkg/jsk_common/issues/1424>`_)

* Contributors: Kei Okada, Kentaro Wada, Yuki Furuta

2.0.17 (2016-07-21)
-------------------
* Remove dependency on python-termcolor
  Fix `#413 <https://github.com/jsk-ros-pkg/jsk_common/issues/413>`_
* Contributors: Kentaro Wada

2.0.16 (2016-06-19)
-------------------
* Fix video recording to avoid the bug in image_view
  https://github.com/ros-perception/image_pipeline/issues/187
* Contributors: Kentaro Wada

2.0.15 (2016-06-13)
-------------------
* Install test files that works properly
* Comment out sanity_lib.py testing on hydro
* Simplify to make sanity_lib work on Travis
* Fix style of code 'test_topic_published.py'
* fix test_topic_published.py to sleep in the beginning when rosbag is used
* Contributors: Kentaro Wada, Yusuke Niitani

2.0.14 (2016-05-14)
-------------------
* Stable ros version check by STRGREATER
* Install cmake directory for executables for catkin
* Support passing command as array
* jsk_tools/src/test_topic_published.py: set default timeout to 10 sec
* jsk_tools/src/sanity_lib.py: add timouetout informatoin
* Contributors: Kei Okada, Kentaro Wada

2.0.13 (2016-04-29)
-------------------
* Fix setting ROS_IP with rossetip on OSX
* Replace slash to create a valid test name
  Modified:
  - jsk_tools/cmake/shell_test.cmake.em
* Contributors: Kentaro Wada

2.0.12 (2016-04-18)
-------------------
* Test tool with shell command with catkin
  Modified:
  - jsk_tools/CMakeLists.txt
  Added:
  - jsk_tools/cmake/run_shell_test.py
  - jsk_tools/cmake/shell_test.cmake.em
* Handle shell and dotfiles for shared computers
  Modified:
  - jsk_tools/CMakeLists.txt
  Added:
  - jsk_tools/env-hooks/99.dotfile.bash
  - jsk_tools/env-hooks/99.dotfile.zsh
  - doc/jsk_tools/cltools/dotfile.rst
* Reuse roscat in rosview shell function
  Modified:
  - jsk_tools/env-hooks/99.jsk_tools.bash
  - jsk_tools/env-hooks/99.jsk_tools.zsh
* Contributors: Kentaro Wada

2.0.11 (2016-03-20)
-------------------

2.0.10 (2016-02-13)
-------------------
* Use "$@" to pass arguments in git-jsk-commit
  Closes https://github.com/jsk-ros-pkg/jsk_common/issues/1319
  Modified:
  - jsk_tools/bin/git-jsk-commit
* Update force_to_rename_changelog_user.py
  https://github.com/jsk-ros-pkg/jsk_common/blob/master/jsk_tools/bin/force_to_rename_changelog_user.py#L58 checks key with lower case, if there is a way to find key with case-insensitive like `(find author autohr_list :test #'(lambda (x y) (str= (lower-case x) (lower-case y)))`, please let me know,
  ```
  author = author.lower()
  if author in REPLACE_RULES:
  replaced_authors.append(REPLACE_RULES[author])
  ``
* [jsk_tools/src/jsk_tools/bag_plotter.py] support yaml field such as [1,2,4-6]
* [jsk_tools/bin/battery_capacity_summary.py] print N/A for non available data
* [bag_plotter.py] Support xlabel and ylabel
* [jsk_tools/bag_plotter] Add label field
  Modified:
  - jsk_tools/src/jsk_tools/bag_plotter.py
* [jsk_tools/bag_plotter] Support messages which does not have header
  Modified:
  - jsk_tools/src/jsk_tools/bag_plotter.py
* [jsk_tools/bin/battery_capacity_summary.py] more battery info
* [jsk_tools/bag_plotter] Add -o to save figure automatically
* [jsk_tools] Add bag file name to the title of plot by bag_plotter.py
* [jsk_tools] Reasonable test result message
  Modified:
  - jsk_tools/src/test_topic_published.py
* [jsk_tools] Add rosview
  Upstream PR: https://github.com/ros/ros/pull/99
  Modified:
  - jsk_tools/env-hooks/99.jsk_tools.bash
  - jsk_tools/env-hooks/99.jsk_tools.zsh
* [jsk_tools] Remove rosrecord: I found this function is not so useful
* Merge pull request `#1309 <https://github.com/jsk-ros-pkg/jsk_common/issues/1309>`_ from wkentaro/git-jsk-commit-markdown-bullet
  [jsk_tools] Add bullet for git-jsk-commit to beautify as markdown
* [jsk_tools] Add topic delay monitor
  upstream PR: `ros/ros_comm#719 <https://github.com/ros/ros_comm/issues/719>`_
  Added:
  jsk_tools/src/topic_delay_monitor.py
* [jsk_tools] Add bullet for git-jsk-commit to beautify as markdown
  Modified:
  - jsk_tools/bin/git-jsk-commit
* [jsk_tools] Add NO_NTP_MONITOR argument to skip ntp monotoring
  in local_pc_monitor.launch
* [jsk_tools] Use jsk-commit for git alias like 'commit-ueda'.
  * Use "$@" in jsk-commit to keep quotes across shell script.
  * Use jsk-commit command for commit-ueda, commit-mmurooka and so on
  Modified:
  jsk_tools/bin/git-jsk-commit
  jsk_tools/src/git_commit_alias.py
* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Ryo KOYAMA, Ryohei Ueda

2.0.9 (2015-12-14)
------------------
* [jsk_tools] test_topic_published.py doesn't work on hydro travis/jenkins
  Modified:
  jsk_tools/CMakeLists.txt
* [jsk_tools] Add roslint_python for jsk_tools
* [jsk_tools] Test test_topic_published.py
* [jsk_tools] Remove [] when not found pkg name
* [jsk_tools] Move dot-files and python library doc
  Modified:
  doc/jsk_tools/index.rst
  jsk_tools/README.md
  Added:
  doc/jsk_tools/dot-files/emacs.md
  doc/jsk_tools/dot-files/tmux.md
* [jsk_tools] Move cl tools from README to sphinx
  Modified:
  doc/index.rst
  jsk_tools/README.md
  Added:
  doc/jsk_tools/cltools/bag_plotter.md
  doc/jsk_tools/cltools/restart_travis.md
  doc/jsk_tools/cltools/rosbag_record_interactive.md
  doc/jsk_tools/cltools/roscore_regardless.md
  doc/jsk_tools/cltools/setup_env_for_ros.md
  doc/jsk_tools/cltools/topic_hz_monitor.md
  doc/jsk_tools/index.rst
  jsk_tools/doc
* [jsk_tools] List added files in git-jsk-commit
* [jsk_tools]
  Modified:
  jsk_tools/bin/git-jsk-commit
* [jsk_tools] git-jsk-commit as git's subcommand
  Usage:
  ```
  git jsk-commit -a
  ```
  Modified:
  jsk_tools/CMakeLists.txt
  jsk_tools/env-hooks/99.jsk_tools.sh
* [jsk_tools] Add wstool info information to report_issue.sh
* [jsk_tools] Add tool to make commit message informative
  This is proposed by @k-okada and discussed on `#1202 <https://github.com/jsk-ros-pkg/jsk_common/issues/1202>`_
  Modified:
  jsk_tools/env-hooks/99.jsk_tools.sh
* [jsk_tools] Add tool to help reporting issue
  It will generate a gist like https://gist.github.com/anonymous/6e1a34227eeb8ef3013c
  See `#1187 <https://github.com/jsk-ros-pkg/jsk_common/issues/1187>`_.
* [jsk_tools/force_to_rename_changelog_user] Add new rule
* [jsk_tools/bag_plotter] Use wxagg for matplotlib backend to speed-up
  plotting
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.8 (2015-12-07)
------------------
* add rostest package.xml
* Contributors: Kei Okada

2.0.7 (2015-12-05)
------------------
* [jsk_tools] Add test for test_stdout.py
* [jsk_tools] Install to share with source permissions
* [jsk_tools] Install to bin/* correctly
* [jsk_tools/bag_plotter] Optimize parsing rosbag file by
  caching accessor
* [jsk_tools] Replace image of topic_hz_monitor
  The command in the image was wrong in previous version.
* [jsk_tools] Fix style of markdown
* [jsk_tools] Use texttable which is released on apt
* [jsk_tools] Add topic_hz_monitor.py
* [jsk_tools] Add kill_after_seconds.py. It will kill a process after
  specified seconds. It is useful to handle roslaunch for benchmarking.
* [jsk_tools] Remove ws_doctor.py
  wstool>=0.1.12 does show equivalent information by ``wstool info``
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

2.0.6 (2015-12-02)
------------------
* [jsk_tools] Add tool to test published topic (check msg comes)
* [jsk_tools] Set parent class as object and return bool in check()
* Contributors: Kentaro Wada

2.0.5 (2015-11-30)
------------------

2.0.4 (2015-11-25)
------------------
* [jsk_topic_tools/rosping_existence] Speak dead nodes
* [jsk_tools] Remove test stdout space, This should be reasonable because rosparam also strip parameter,   automatically.
* [jsk_tools] Warning about designed for test.  After long discussion at `#1216 <https://github.com/jsk-ros-pkg/jsk_common/issues/1216>`_
* [jsk_tools] test_stdout.py tests each lines
* [jsk_tools] Add delay_timestamp.py
* [jsk_tools] Install run_tmux for gdb debugging. That is described here:  http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB
* [jsk_tools] Add rosrecord shell function
* [jsk_tools] Set calc_md5.py to correct dir (src)
* [jsk_tools] Add ~shell param for test_stdout.py
* [jsk_tools] FIx dot.emacs to run euslisp correctly
* [jsk_tools] Add test utility node test_stdout
* [jsk_tools] Add ntp_monitor to local_pc_monitor
* [jsk_tools] add rosbag_record_interactive. select topic using zenity and record them
* [jsk_tools] show minorticks and grid
* [jsk_tools] Correct order of ROS_IP in list of hostname -I.  Closes `#1170 <https://github.com/jsk-ros-pkg/jsk_common/issues/1170>`_
* [jsk_tools] Add document about roscore_regardless.py
* [jsk_tools] Commandline tool for selection with percol
* [jsk_tools] Add completion for restart_travis
* [jsk_tools] Add documentation for restart_travis
* [jsk_tools] Add restart_travis function
* [jsk_tools] Disable vi-mode in tmux
* [jsk_tools] Add document about tmux.conf
* [jsk_tools] New users to force_to_rename_changelog_user.py.
* Remove no need stdout in rossetip
* [jsk_tools] Add document about inferior-lisp-mode
* [jsk_tools] Write to stderr when rossetip fails
* [jsk_tools] Do not create duplicated inferior-lisp buffer
* [jsk_tools/force_to_rename_changelog_user.py] New 3 users
* [jsk_tools] Use keyboard to toggle legend
* [jsk_common/bag_plotter] Optimize bag parsing speed by topics keyword of read_messages method
* [jsk_tools] Add rosemacs-el to dependency
* [jsk_tools/bag_plotter] Synchronize x axis zoom/pan and add cheap button to toggle legend
* [jsk_tools/bag_plotter] Toggle legend by clicking
* [jsk_tools/bag_plotter] Support manual layout of figures
* [jsk_tools/bag_plotter] Support plotting of array
* [jsk_tools/bag_plotter] Support multiple bag files
* [jsk_tools/bag_plotter.py] Support --duration and --start option
* [jsk_tools/bag_plotter] Use interactive mode of matplotlib to enable Ctrl-C
* [jsk_tools] use hostname to search ip
* [jsk_tools] Add dot-files directory, which is copied from JSK internal svn, to share common setup in shared-users
* Contributors: Eisoku Kuroiwa, Yuki Furuta, Kentaro Wada, Ryohei Ueda

2.0.3 (2015-07-24)
------------------
* [jsk_tools/99.jsk_tools.sh] fix typo
* [jsk_tools/99.jsk_tools.sh] Safer rost func and support rosmsg show
* [jsk_tools/99.jsk_tools.sh] Safer rosn function when selecting in percol
* [jsk_tools/99.jsk_tools.sh] depends should be resolved via rosdep install
* [jsk_tools] Add bag_plotter.py to README
* [jsk_tools] Add plotting code from bag file
* [jsk_tools] Fix to use lsof to lookup CLOSE_WAIT num
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.2 (2015-07-07)
------------------
* [jsk_tools] Remove monitor_roscore.py
* [jsk_tools] Add monitoring script to check roscore CLOSE_WAIT num
* [jsk_tools] Check msg type is same as published one
* [jsk_tools] import sanity_lib in __init__.py
* [jsk_tools] Add network stats to local_pc_monitor.launch
* Contributors: Kentaro Wada, Ryohei Ueda

2.0.1 (2015-06-28)
------------------
* [jsk_tools] Add local_pc_monitor.launch to monitor load of computers
* Contributors: Ryohei Ueda

2.0.0 (2015-06-19)
------------------
* [jsk_tools] Record image_rect of axis camera
* [jsk_tools] Add calibration data
* [jsk_tools] Add launch to record axis camera
* Contributors: Kentaro Wada

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
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Iori Kumagai

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
