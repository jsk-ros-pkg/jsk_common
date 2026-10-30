# jsk_ros1_ros2_compat

Shared ROS1 (roscpp/rospy) / ROS2 (rclcpp/rclpy) compatibility helpers, so
that a package can keep **one** source tree that builds and runs on both,
with the ROS1 code left as close to the original as possible.

## Porting style

- One `package.xml` / `CMakeLists.txt` branched on `$ROS_VERSION`.
  - `package.xml`: `condition="$ROS_VERSION == 1"` / `"== 2"` on the
    dependencies and on `<build_type>` in `<export>`.
  - `CMakeLists.txt`: keep the ROS1 lines byte-identical (no
    re-indentation), e.g. prepend the ROS2 build and wrap the original in
    `else()`/`endif()`:

    ```cmake
    if("$ENV{ROS_VERSION}" EQUAL 2)
      cmake_minimum_required(VERSION 3.8)
      project(foo)
      find_package(ament_cmake REQUIRED)
      find_package(jsk_ros1_ros2_compat REQUIRED)
      jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/foo_node.py)
      ament_package()
    else() # if("$ENV{ROS_VERSION}" EQUAL 2)
    cmake_minimum_required(VERSION 2.8.3)
    project(foo)
    ...  # the ROS1 original, unchanged
    endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
    ```
- Python nodes call `ros_compat.xxx(node, ...)` instead of `rospy.xxx(...)`,
  where `node` is `None` on ROS1 and the `rclpy.node.Node` on ROS2.

## Python: `jsk_ros1_ros2_compat.rospy_rclpy_compat`

```python
from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION

if __name__ == '__main__':
    if ROS_VERSION == 2:
        ros_compat.rclpy.init()
        node = ros_compat.rclpy.create_node('foo')
        Foo(node)
        ros_compat.spin_and_shutdown(node)
    else:
        rospy.init_node('foo')
        Foo()
        rospy.spin()
```

| | ROS1 | ROS2 |
|---|---|---|
| `get_param(node, '~name', default)`, `has_param` | `rospy.get_param` | declares a (dynamically typed) parameter |
| `loginfo/logwarn/logerr/logdebug/logfatal(node, msg)` | `rospy.log*` | `node.get_logger().*` (msg must be a `str`) |
| `loginfo/logwarn/logerr_throttle(node, period, msg)` | `rospy.log*_throttle` | `throttle_duration_sec=period` |
| `create_publisher(node, topic, type, queue_size=1)` | `rospy.Publisher` | `node.create_publisher` |
| `create_subscription(node, topic, type, cb, queue_size=None)` | `rospy.Subscriber` (`None`: unbounded) | `node.create_subscription` (`None`: depth 1) |
| `create_service`, `create_trigger_service`, `call_service`, `create_request/response` | `rospy.Service/ServiceProxy` | `node.create_service/create_client` |
| `create_timer(node, period, cb)` | `rospy.Timer` | `node.create_timer` (`cb(None)`) |
| `create_simple_action_client(node, name, type)` | `actionlib.SimpleActionClient` | same subset over `rclpy.action.ActionClient` |
| `create_simple_action_server(node, name, type, execute_cb)` | `actionlib.SimpleActionServer` | same subset over `rclpy.action.ActionServer` |
| `create_reconfigure_server(node, Config, cb, defaults)` | `dynamic_reconfigure.server.Server` | parameters declared from `defaults` + on-set callback |
| `now_sec`, `stamp_now`, `stamp_to_sec`, `duration_from_sec`, `sleep`, `Rate` | `rospy.Time/Duration/...` | `node.get_clock()`, ... |
| `get_name`, `is_shutdown`, `on_shutdown`, `signal_shutdown` | `rospy.*` | node name / rclpy context |
| `create_tf_buffer_and_listener`, `create_tf_broadcaster` | `tf2_ros` | `tf2_ros` with `node` |
| `spin_and_shutdown(node, executor=None)` | - | spin until SIGINT/SIGTERM/`signal_shutdown`, then clean up |

Action types: pass `FooAction`/`FooGoal` on ROS1 and `Foo`/`Foo.Goal` on
ROS2; timeouts are in seconds on both. `actionlib`/`dynamic_reconfigure`
are imported lazily, so depend on them in your own package.

### Packages outside jsk_common

jsk_ros1_ros2_compat is only released with jsk_common >= 3.0, which is not
available for every ROS1 distro. Depend on it for ROS2 only and fall back
to rospy when it can not be imported, listing just the functions you use:

```xml
<exec_depend condition="$ROS_VERSION == 2">jsk_ros1_ros2_compat</exec_depend>
```

```python
try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
```

## C++: `#include <jsk_ros1_ros2_compat/compat.h>`

ROS1/ROS2 detection, `ROS_INFO`/`ROS_ERROR`/... and `ros::param::param()`
for rclcpp, `RosTime`/`RosDuration`/`rosTimeNow()`, `RosPublisher<T>`/
`RosSubscriber<T>`/`publishMsg()`, and message alias macros. See the
comments in each header under `include/jsk_ros1_ros2_compat/`.

## CMake (ROS2)

Available after `find_package(jsk_ros1_ros2_compat REQUIRED)`; add
`<buildtool_depend condition="$ROS_VERSION == 2">jsk_ros1_ros2_compat</buildtool_depend>`.

### `jsk_ros1_ros2_compat_install_python()`

```cmake
jsk_ros1_ros2_compat_install_python(
  PROGRAMS node_scripts/foo.py ...
  [DESTINATION lib/${PROJECT_NAME}]   # default, for `ros2 run`
  [USE_VENV])
```

Installs scripts that keep `#!/usr/bin/env python` for ROS1 (python2 on
indigo..melodic) with a `python3` shebang (ROS2 base images have no bare
`python`), or the python3 of the venv below with `USE_VENV`.

### `jsk_ros1_ros2_compat_generate_uv_venv()`

The ROS2 counterpart of catkin_virtualenv's `catkin_generate_virtualenv()`
(catkin_virtualenv has no ROS2 release), defined in `cmake/catkin_uv.cmake`.

```cmake
jsk_ros1_ros2_compat_generate_uv_venv(
  [PYTHON_INTERPRETER python3]
  [NO_SYSTEM_SITE_PACKAGES]
  [CHECK_IMPORTS module ...]
  [STRIP])
jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/foo.py USE_VENV)
```

Creates a relocatable venv from the package's `pyproject.toml`/`uv.lock`
with `uv sync --frozen` at configure time and installs it to
`share/${PROJECT_NAME}/venv` (so it is shipped in the deb). System
site-packages, such as rclpy and messages, stay visible unless
`NO_SYSTEM_SITE_PACKAGES`. Sets `${PROJECT_NAME}_VENV_DIR` (build tree) and
`${PROJECT_NAME}_VENV_INSTALL_DIR` (install tree).

uv has neither a rosdep key nor an Ubuntu package, so this package
installs a pinned prebuilt uv to `lib/jsk_ros1_ros2_compat/uv` at its own
build time, and the function uses that one. Create `uv.lock` with
`uv lock` next to `pyproject.toml`, e.g.

```toml
[project]
name = "foo"
version = "0.0.0"
requires-python = ">=3.10"
dependencies = ["six==1.16.0"]

[tool.uv]
package = false
```

ROS1 keeps using catkin_virtualenv; these uv helpers are not defined there.
