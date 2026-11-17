# jsk_tools

Everyday tools for working with ROS robots in JSK.
The package provides a node that speaks diagnostics errors (`audible_warning.py`), command line tools to monitor topics,
hosts and roscore, test nodes for `rostest`, shell functions to set up `ROS_MASTER_URI` / `ROS_IP`, and dot files for emacs and tmux.

## ROS 1 / ROS 2

The package builds for both ROS 1 (catkin) and ROS 2 (ament_cmake); `CMakeLists.txt` branches on `$ENV{ROS_VERSION}`.

| Item | ROS 1 | ROS 2 |
|---|---|---|
| `audible_warning.py` (`node_scripts/`) | yes | yes |
| `battery_capacity_summary.py` (`bin/`) | yes | yes |
| Python module `jsk_tools` | yes | yes |
| Other tools in `bin/` (`roscore_regardless.py`, `bag_plotter.py`, `check_host_sanity.py`, `kill_after_seconds.py`, `roscore_check.py`, `roscore_monitor.py`, `ros_console.py`, `run_tmux`, `git-jsk-commit`, ...) | yes | - |
| Tools in `src/` (`topic_hz_monitor.py`, `topic_delay_monitor.py`, `rostopic_host_sanity`, `sanity_diagnostics.py`, `nodes_graph.py`, ...) | yes | - |
| Test nodes `test_topic_published.py`, `test_rosparam_set.py`, `test_stdout.py` | yes | - |
| Shell env-hooks (`rossetmaster`, `rossetip`, `config_switch`, ...) and CMake extras (`jsk_tools_add_shell_test`, `download_package.cmake`) | yes | - |
| Launch files in `launch/` and `sample/` | yes | - |

The rostest helpers, `sanity_diagnostics.py` and the `roscore_*` tools rely on ROS 1 master introspection and stay ROS 1 only.
There are no ROS 2 launch files; use `ros2 run jsk_tools <exe>`.

## audible_warning.py

Subscribes `/diagnostics_agg` (`diagnostic_msgs/DiagnosticArray`) and speaks the error contents through the
`/robotsound` action (`sound_play/SoundRequestAction`) on ROS 1; remap it if your sound server has another name.
On ROS 2 it does not speak, as there is no `sound_play` of ROS 2: it logs `[TTS disabled under ROS2] would speak: ...`
and publishes the text (`~output/text`, `~output/original_text`) only.

Main parameters: `~speak_interval` (default `120.0` s, the same error is not repeated within this time), `~volume`, `~language`,
`~wait_speak`, `~seconds_to_start_speaking`, `~enable`, `~speak_ok` / `~speak_warn` / `~speak_error` / `~speak_stale`,
`~run_stop_topic` and `~run_stop_condition` (silence errors while the robot is run-stopped), `~blacklist` and `~run_stop_blacklist`
(regular expressions on diagnostic `name` / `message`).
See [audible_warning.md](../doc/jsk_tools/scripts/audible_warning.md) for the full list.

On ROS 2, `~blacklist` / `~run_stop_blacklist` are not supported (ROS 2 parameters cannot hold lists of dicts) and
`~run_stop_topic` is always subscribed as `std_msgs/Bool`. The `enable`, `speak_*` parameters can be changed at runtime
(dynamic_reconfigure on ROS 1, `ros2 param set` on ROS 2).

```bash
# ROS 1
roslaunch jsk_tools sample_audible_warning.launch
# ROS 2 (no speech, the log and the text only)
ros2 run jsk_tools audible_warning.py --ros-args -p speak_interval:=60.0
```

## battery_capacity_summary.py

Subscribes `/diagnostics_agg` and prints a table of the `/Power System/Smart Battery` entries
(serial number, manufacture date, full/remaining capacity, voltage, cycle count, status). Requires `colorama`.

```bash
rosrun jsk_tools battery_capacity_summary.py         # ROS 1
ros2 run jsk_tools battery_capacity_summary.py       # ROS 2
```

## Command line tools (ROS 1)

* **topic_hz_monitor.py** -- finds the bottleneck in a chain of nodes by showing the rate of a topic and, with `--search-parent`,
  of its upstream topics. [doc](../doc/jsk_tools/cltools/topic_hz_monitor.md)
* **topic_delay_monitor.py** -- same as `topic_hz_monitor.py`, but shows the delay from the header timestamp. [doc](../doc/jsk_tools/cltools/topic_delay_monitor.md)
* **rostopic_host_sanity** -- checks the network connection to every host in the ROS topic graph. [doc](../doc/jsk_tools/cltools/rostopic_host_sanity.rst)
* **roscore_regardless.py** -- runs a command, killing and restarting it according to roscore liveness. [doc](../doc/jsk_tools/cltools/roscore_regardless.md)
* **bag_plotter.py** -- plots fields of a bag file directly, configured by a yaml file. [doc](../doc/jsk_tools/cltools/bag_plotter.md)
* **sanity_diagnostics.py** -- publishes to `/diagnostics` whether the topics and nodes listed in `~sanity_targets` are alive. [doc](../doc/jsk_tools/scripts/sanity_diagnostics.md)

```bash
rosrun jsk_tools topic_hz_monitor.py /euclidean_segmentation/output --search-parent
rosrun jsk_tools roscore_regardless.py rostopic echo /foo
```

## Shell functions (ROS 1)

Installed as catkin env-hooks, available after sourcing the workspace:

* `rossetmaster [hostname] [port]`, `rossetlocal`, `rosdefault` / `rossetdefault` -- set `ROS_MASTER_URI`.
* `rossetip` -- set `ROS_IP` and `ROS_HOSTNAME`.
* `config_switch` -- switch dotfiles and shell between users.

See [setup_env_for_ros.md](../doc/jsk_tools/cltools/setup_env_for_ros.md) and [config_switch.rst](../doc/jsk_tools/cltools/config_switch.rst).

## Test nodes (ROS 1)

For use in `rostest` files:

* **test_topic_published.py** -- checks that topics (`~topic_%d`) are (or, with `~negative_%d`, are not) published within `~timeout_%d`,
  optionally checking a Python `~condition_%d`. [doc](../doc/jsk_tools/test_nodes/test_topic_published.rst)
* **test_stdout.py** -- checks the stdout of a shell command (`~command`, `~stdout`, `~stdout_line%d`). [doc](../doc/jsk_tools/test_nodes/test_stdout.rst)
* **test_rosparam_set.py** -- checks that parameters are set.

```xml
<test test-name="test_stdout" pkg="jsk_tools" type="test_stdout.py">
  <param name="command" value="echo hello" />
  <param name="stdout_line0" value="hello" />
  <param name="shell" value="true" />
</test>
```

## Launch files (ROS 1)

* `sample/sample_audible_warning.launch` -- `audible_warning.py` with pseudo diagnostics and a pseudo run-stop publisher.
* `sample/sample_sanity_diagnostics.launch` -- `sanity_diagnostics.py` sample.
* `launch/local_pc_monitor.launch` -- monitors the local PC (CPU, NTP via `pr2_computer_monitor`, network via `jsk_network_tools`).
* `launch/record_axis_camera.launch` -- records video from an Axis network camera with `image_view/video_recorder`.

## Documentation

See [doc/jsk_tools](../doc/jsk_tools/index.rst): [command line tools](../doc/jsk_tools/cltools), [test nodes](../doc/jsk_tools/test_nodes),
[Python library](../doc/jsk_tools/python/sanity_lib.md), [scripts](../doc/jsk_tools/scripts) and [dot files](../doc/jsk_tools/dot-files).
