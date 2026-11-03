^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_ros1_ros2_compat
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (2026-11-03)
------------------
* Merge pull request `#1835 <https://github.com/jsk-ros-pkg/jsk_common/issues/1835>`_ from k-okada/ros-version-fallback-ament
  Fix ROS2 buildfarm binary jobs: ROS_VERSION without ros_environment, boost for jsk_topic_tools
* Merge pull request `#1834 <https://github.com/jsk-ros-pkg/jsk_common/issues/1834>`_ from pazeshun/array-to-heatmap
  [jsk_ros1_ros2_compat, jsk_topic_tools, jsk_rosbag_tools] Add array heatmap tools with ROS 1 and ROS 2 support
* Guess ROS_VERSION from AMENT_PREFIX_PATH when unset, depend on ros_environment
  The ROS2 binary jobs of the buildfarm failed at find_package(catkin): they
  install only the build dependencies, none of which pulls in
  ros_environment, so $ENV{ROS_VERSION} is empty and the fallback at the top
  of CMakeLists.txt chose ROS1.
  - package.xml: add a ROS2 buildtool_depend on ros_environment, which sets
  ROS_VERSION.
  - CMakeLists.txt: when ROS_VERSION is still unset, guess it from
  AMENT_PREFIX_PATH (bloom's ROS2 debian/rules passes it to cmake,
  setup.sh and colcon set the environment variable, catkin never does)
  instead of always assuming ROS1, and warn about it. jsk_tools had no
  fallback at all and gets the same block.
  Applies to jsk_ros1_ros2_compat, jsk_tools, jsk_topic_tools, jsk_common,
  jsk_data, jsk_rosbag_tools, dynamic_tf_publisher, audio_video_recorder and
  image_view2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request `#1 <https://github.com/jsk-ros-pkg/jsk_common/issues/1>`_ from k-okada/array-to-heatmap-old-distros
  Support indigo/kinetic in array heatmap tools (OpenCV 2.4, numpy < 1.9, rosbag < 1.13)
* [jsk_ros1_ros2_compat] rosbag: raw messages with connection headers on rosbag < 1.13
  rosbag of indigo/kinetic has neither read_messages(...,
  return_connection_header=) nor write(..., connection_header=), which
  failed test_add_heatmap on the kinetic CI. There, read_raw_messages()
  takes the header of each topic's connection from the bag, and
  write_raw() registers a new topic's connection with the given header
  before writing, as write(..., connection_header=) does on newer rosbag.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Add array heatmap tools with ROS 1 and ROS 2 support
  - Render numeric MultiArray and scalar Image topics as heatmap images with optional cell values and configurable color scales
  - Add heatmap topics to bags while preserving original messages, recording timestamps, and connection or QoS metadata
  - Extend the compatibility layer with raw bag I/O and shared message type helpers
  - Add installation rules, documentation, and tests for both ROS versions
* Merge pull request `#1833 <https://github.com/jsk-ros-pkg/jsk_common/issues/1833>`_ from k-okada/release-3.0.2
  3.0.2
* Contributors: Kei Okada, Shun Hasegawa

3.0.2 (2026-11-02)
------------------
* Merge pull request `#1831 <https://github.com/jsk-ros-pkg/jsk_common/issues/1831>`_ from k-okada/ros2-compat-fixes
  [jsk_ros1_ros2_compat] fix queue_size/shutdown/throttle; add init_node/spin, latch, actionlib, reconfigure, uv venv helpers
* [jsk_ros1_ros2_compat] spin() with an own executor, not rclpy's global one
  rclpy.spin(node) spins rclpy's global executor, so a callback could not
  call a library which spins its own node with
  rclpy.spin_until_future_complete(), such as sound_play's
  SoundClient(node, blocking=True).say() ('Executor is already
  spinning').
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] return from wait_for_result() on shutdown
  On ROS2, the result of a goal never comes after SIGINT/SIGTERM, so a
  callback blocked in send_goal_and_wait()/wait_for_result() without a
  timeout kept its executor thread, and so the process, alive.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add init_node(..., multi_threaded=True)
  For nodes whose callbacks wait for other callbacks (e.g. a service
  callback waiting for a topic or an action result), which works on
  rospy, as it runs each callback in its own thread, but deadlocks with
  rclpy's single-threaded executor and default (mutually exclusive)
  callback group. With multi_threaded=True, create\_\*() of the node
  default to a ReentrantCallbackGroup and spin() uses a
  MultiThreadedExecutor, so the node code needs no rclpy specific
  executor/callback group; ignored on ROS1.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add callback_group to create\_\* (ROS2 only)
  create_subscription/service/trigger_service/timer and
  create_simple_action_client/server take callback_group=, passed to
  rclpy (e.g. a ReentrantCallbackGroup with a MultiThreadedExecutor) and
  ignored by rospy, whose callbacks each run in their own thread.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add log\*_once, throttle per call site
  rospy's log\*_throttle/log\*_once and rclpy's throttle_duration_sec/once
  tell call sites apart by the calling frame, which for the
  log\*_throttle() wrappers here was always the same line in this module,
  so e.g. a second throttled message from another place in a node was
  never logged. Do the throttling in compat, keyed on the caller's file
  and line, and add loginfo/logwarn/logerr_once() on top of it (rospy's
  log\*_once, which indigo's rospy lacks, or rclpy's once=True).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add wait_for_service/set_param, map ~ in action names
  - wait_for_service(node, name, srv_type, timeout): returns whether the
  service became available (rospy raises on timeout).
  - set_param(node, name, value): rospy.set_param(), or set (declaring it
  first if needed) the node parameter on ROS2.
  - create_simple_action_server()/create_simple_action_client() map
  ~foo to ~/foo on ROS2, like create_publisher/subscription/service.
  Packages in jsk_3rdparty otherwise kept ROS_VERSION branches for these.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add latch, get_response_class, init_node/spin/spin_once; log without node; run on_shutdown hooks on signals
  Found while porting jsk_3rdparty packages, where each of these was
  written per package with ROS_VERSION branches:
  - create_publisher(..., latch=True): rospy's latched topic, or a
  transient-local QoS publisher; create_subscription(..., latch=True)
  subscribes with the matching QoS on ROS2.
  - get_response_class(srv_type): FooResponse on ROS1, or a callable
  building Foo.Response that also takes the fields positionally (in
  .srv order), so FooResponse(\*values) keeps working on ROS2. None
  values are left at their defaults, as rclpy rejects them.
  - init_node(name)/spin(node)/spin_once(node, timeout_sec)/shutdown(node),
  so that a main() needs no ROS_VERSION branch: init_node() returns None
  on ROS1 and a new rclpy node on ROS2; spin_once() returns quietly after
  SIGINT/SIGTERM, and shutdown() does spin()'s cleanup for nodes with
  their own while-not-is_shutdown loop.
  - create_subscription()/create_service() map ~foo to ~/foo on ROS2 like
  create_publisher() already did.
  - log\*(None, msg): on ROS2, without a node (e.g. at import time), log
  with a logger named after the script.
  - on_shutdown() hooks were not called on ROS2 when the node exited on
  SIGINT/SIGTERM: rclpy's signal handlers shut the context down in C
  without running its python hooks. spin_and_shutdown() now runs them
  (once). It also no longer raises when a signal arrives before the
  executor starts spinning.
  - README: the API additions, and the alternative of keeping the
  original rospy code as is and adding a compat version next to it.
  Tested with test/test_rospy_rclpy_compat.py on jazzy (13 tests) and ROS-O (11 tests).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add README
  Porting style, the Python/C++/CMake APIs, and the ImportError fallback
  for packages outside jsk_common that also have to build against
  jsk_common < 3.0 on ROS1.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add tests of rospy_rclpy_compat
  test/test_rospy_rclpy_compat.py is shared by ROS1 (rostest) and ROS2
  (ament_cmake_pytest): params, logging (incl. throttle/fatal), get_name,
  pub/sub (incl. ROS1 queue_size passthrough), the simple action
  client/server pair with a Fibonacci action (goal, feedback, cancel),
  the ROS2 reconfigure server, and on_shutdown/signal_shutdown ending
  spin_and_shutdown().
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add uv venv CMake helper (catkin_uv.cmake)
  - cmake/catkin_uv.cmake: jsk_ros1_ros2_compat_generate_uv_venv(), the
  ROS2 counterpart of catkin_virtualenv, which has no ROS2 release.
  Creates a relocatable venv (with system site-packages, so rclpy etc.
  stay visible) from the package's pyproject.toml/uv.lock with
  `uv sync --frozen` and installs it to share/${PROJECT_NAME}/venv, so
  that it is also shipped in the deb. Based on voicevox's ROS2 port by
  Michi-Tsubaki (`jsk-ros-pkg/jsk_3rdparty#573 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/573>`_).
  - uv has neither a rosdep key nor an Ubuntu package, so on ROS2 this
  package installs a pinned prebuilt uv (0.11.28) with uv's installer
  (via curl, a new build_depend) to lib/jsk_ros1_ros2_compat/uv, and
  catkin_uv.cmake uses that one. Packages using it need no network or
  extra dependency for uv itself.
  - jsk_ros1_ros2_compat_install_python(... USE_VENV) installs scripts with
  the venv's python3 as their shebang.
  - ROS2 only: ROS1 keeps using catkin_virtualenv, and catkin_uv.cmake is
  not included there.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat] add actionlib/dynamic_reconfigure equivalents and on_shutdown
  - create_simple_action_client()/create_simple_action_server():
  actionlib.SimpleActionClient/SimpleActionServer (ROS1) or small
  wrappers of rclpy's ActionClient/ActionServer with the same commonly
  used subset (wait_for_server, send_goal[_and_wait], wait_for_result,
  get_result, cancel_goal / start, publish_feedback,
  is_preempt_requested, set_succeeded/aborted/preempted). Timeouts are
  in seconds on both. jsk_3rdparty's dialogflow_task_executive,
  ros_google_cloud_language and switchbot_ros each carried their own
  copy of such a wrapper.
  - create_reconfigure_server(): dynamic_reconfigure.server.Server (ROS1),
  or node parameters declared from a defaults dict plus an
  on-set-parameters callback calling the same callback(config, level)
  (ROS2), with config usable as config.name or config['name'].
  - on_shutdown(): rospy.on_shutdown() or the rclpy context's hook.
  actionlib/dynamic_reconfigure are imported lazily, so packages using
  these depend on them themselves.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [jsk_ros1_ros2_compat, jsk_tools, jsk_data] fix queue_size/shutdown handling, add missing helpers
  - create_subscription(): ROS1 silently dropped queue_size, so e.g.
  audible_warning.py's queue_size=1 subscribers became unbounded. Pass
  it through, and default to None so each side keeps its own default
  (unbounded for rospy, depth 1 for rclpy). battery_capacity_summary.py
  and data_collection_server.py passed 1 only because of the old
  default; drop it so their ROS1 queues stay unbounded as before.
  - spin_and_shutdown(): rclpy's signal handlers raise
  ExternalShutdownException (not KeyboardInterrupt) out of spin() on
  SIGINT/SIGTERM, which printed a traceback and exited with 1. Treat it
  as a normal exit, and use rclpy.try_shutdown().
  - add logfatal(), log{info,warn,err}_throttle(), get_name() and
  signal_shutdown(), which ROS2-ported packages in jsk_3rdparty
  otherwise each had to define themselves.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request `#1832 <https://github.com/jsk-ros-pkg/jsk_common/issues/1832>`_ from k-okada/ros2-compat-install-python
  [jsk_ros1_ros2_compat] add jsk_ros1_ros2_compat_install_python() and use it
* [jsk_ros1_ros2_compat] add jsk_ros1_ros2_compat_install_python() CMake helper
  Exported as CONFIG_EXTRAS (ament) / CFG_EXTRAS (catkin).
  jsk_ros1_ros2_compat_install_python(PROGRAMS ... [DESTINATION ...])
  installs scripts that keep the `#!/usr/bin/env python` shebang shared
  with ROS1 (python2 on indigo..melodic) with a python3 shebang, since
  ROS2 base images ship no bare `python`. The copy is regenerated when
  the source changes. This replaces the foreach(file(READ)/file(WRITE))
  block repeated in every ROS2-ported package, which ran at configure
  time only, so a rebuild after editing a script installed the stale copy.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

3.0.0 (2026-10-29)
------------------
* Merge pull request `#1826 <https://github.com/jsk-ros-pkg/jsk_common/issues/1826>`_ from k-okada/ros2-support
  Add ROS2 support to jsk_ros1_ros2_compat, dynamic_tf_publisher, jsk_topic_tools, jsk_tools, audio_video_recorder, jsk_data, image_view2, jsk_rosbag_tools
* [jsk_ros1_ros2_compat] new package: shared ROS1/ROS2 compat helpers
  Consolidates the ROS1/ROS2 compatibility boilerplate needed by this
  branch's ported packages, so each package doesn't maintain its own
  near-duplicate copy:
  - include/jsk_ros1_ros2_compat/compat.h: the __has_include(<ros/ros.h>)
  self-detection block, the ROS1_ROS2_COMPAT namespace (RosTime/
  RosDuration/rosTimeNow()/RosPublisher<T>/RosSubscriber<T>/
  RosServiceServer<T>/publishMsg()/topicName(), backed by a single
  g_node set once per process), the ROS_INFO/ROS_ERROR/ROS_WARN/
  ROS_DEBUG (+_STREAM) logging-macro shim, and ros::param::param<T>()
  reopened under ROS2.
  - src/jsk_ros1_ros2_compat/rospy_rclpy_compat.py: an rospy/rclpy compat
  shim (get_param, log*, create_publisher/subscription/service, ...)
  for packages using rclpy under ROS2.
  - src/jsk_ros1_ros2_compat/log_utils.py: jsk_log*()/*_throttle()
  logging helpers on top of rospy_rclpy_compat's node-registration
  pattern.
  This package has no dependencies on any other jsk_common package, so
  depending on it never risks a dependency cycle. Consuming packages are
  migrated to it in the following commits.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
* Contributors: Kei Okada
