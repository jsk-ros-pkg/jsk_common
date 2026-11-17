# jsk_ros1_ros2_compat

Shared ROS1 (roscpp/rospy) / ROS2 (rclcpp/rclpy) compatibility helpers, so
that a package can keep **one** source tree that builds and runs on both,
with the ROS1 code left as close to the original as possible.

## ROS 1 / ROS 2

This is a library package (no nodes, scripts or launch files). It builds for
both ROS 1 (catkin) and ROS 2 (ament), selected by `$ROS_VERSION`:

| | ROS 1 | ROS 2 |
|---|---|---|
| C++ headers (`include/jsk_ros1_ros2_compat/*.h`) | yes | yes |
| Python package `jsk_ros1_ros2_compat` (`rospy_rclpy_compat`, `rosbag`, `log_utils`) | yes | yes |
| CMake `jsk_ros1_ros2_compat_install_python()` | defined, but meant for ROS 2 builds | yes |
| CMake `jsk_ros1_ros2_compat_generate_uv_venv()` and the bundled `uv` binary (`lib/jsk_ros1_ros2_compat/uv`) | no (use catkin_virtualenv) | yes |

`ROS_VERSION` in the Python modules is read from the `ROS_VERSION`
environment variable (default `1`). The C++ headers detect ROS 1 by whether
`<ros/ros.h>` can be included, so no compile definition is needed.

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
    node = ros_compat.init_node('foo')  # None on ROS1
    Foo(node)
    ros_compat.spin(node)
```

| | ROS1 | ROS2 |
|---|---|---|
| `get_param(node, '~name', default)`, `has_param` | `rospy.get_param` | declares a (dynamically typed) parameter |
| `init_node(name, anonymous=False, multi_threaded=False)` | `rospy.init_node`, returns `None` | `rclpy.init()` + a new node, returned (`multi_threaded`: see below) |
| `spin(node)`, `spin_once(node, timeout_sec)`, `shutdown(node)` (after an own `while not is_shutdown(node)` loop) | `rospy.spin`, `rospy.sleep`, - | `spin_and_shutdown`, `rclpy.spin_once`, cleanup |
| `loginfo/logwarn/logerr/logdebug/logfatal(node, msg)` | `rospy.log*` | `node.get_logger().*` (msg must be a `str`; `node=None` uses a logger named after the script) |
| `loginfo/logwarn/logerr_throttle(node, period, msg)`, `loginfo/logwarn/logerr_once(node, msg)` | like `rospy.log*_throttle/_once` | like `throttle_duration_sec=/once=True` (per call site, done in compat) |
| `create_publisher(node, topic, type, queue_size=1, latch=False)` | `rospy.Publisher` | `node.create_publisher` (depth `queue_size`, default reliable QoS; `latch`: transient-local QoS; `~foo` -> `~/foo`) |
| `create_subscription(node, topic, type, cb, queue_size=None, latch=False, callback_group=None, qos=None)` | `rospy.Subscriber` (`None`: unbounded; `latch`, `qos` ignored) | `node.create_subscription` (`None`: depth 1; QoS: see below; `~foo` -> `~/foo`) |
| `wait_for_service(node, name, type, timeout)` (returns bool), `set_param(node, name, value)` | `rospy.wait_for_service`, `rospy.set_param` | a client's `wait_for_service`, `node.set_parameters` |
| `create_service`, `create_trigger_service`, `call_service`, `create_request/response`, `get_response_class` (`FooResponse(*values)` on both) | `rospy.Service/ServiceProxy` | `node.create_service/create_client` |
| `create_timer(node, period, cb)` | `rospy.Timer` | `node.create_timer` (`cb(None)`) |
| `create_simple_action_client(node, name, type)` | `actionlib.SimpleActionClient` | same subset over `rclpy.action.ActionClient` |
| `create_simple_action_server(node, name, type, execute_cb)` | `actionlib.SimpleActionServer` | same subset over `rclpy.action.ActionServer` |
| `callback_group=` of `create_subscription/service/trigger_service/timer/simple_action_client/simple_action_server` | ignored (rospy runs each callback in its own thread) | the rclpy callback group, e.g. `ReentrantCallbackGroup()` with a `MultiThreadedExecutor` |
| `init_node(name, multi_threaded=True)`, for callbacks waiting for other callbacks (e.g. a service callback waiting for a topic) | ignored (as above) | `create_*` default to a `ReentrantCallbackGroup` and `spin()` uses a `MultiThreadedExecutor` |
| `create_reconfigure_server(node, Config, cb, defaults)` | `dynamic_reconfigure.server.Server` | parameters declared from `defaults` + on-set callback |
| `now_sec`, `stamp_now`, `stamp_to_sec`, `duration_from_sec`, `sleep`, `Rate` | `rospy.Time/Duration/...` | `node.get_clock()`, ... |
| `get_name`, `is_shutdown`, `on_shutdown`, `signal_shutdown` | `rospy.*` | node name / rclpy context |
| `create_tf_buffer_and_listener`, `create_tf_broadcaster` | `tf2_ros` | `tf2_ros` with `node` |
| `spin_and_shutdown(node, executor=None)` | - | spin until SIGINT/SIGTERM/`signal_shutdown`, then clean up |
| `private_name(name)`, `message_type_name(msg)`, `resolve_message_class('pkg/Type')` | name unchanged, `msg._type`, `roslib.message` | `~foo` -> `~/foo`, `'pkg/Type'`, `get_message` (accepts `pkg/Type` and `pkg/msg/Type`) |
| `declare_params(node, defaults)`, `get_params(node, names)` | - | ROS2 only: declare / read several parameters at once |

QoS of `create_subscription()` on ROS2 (ignored on ROS1), from the `qos`
argument:

| `qos` | QoS |
|---|---|
| `None` (default) | `'any'`, or transient-local with depth `queue_size` if `latch=True` |
| `'any'` | BEST_EFFORT + VOLATILE with depth `queue_size`: reads the topic of any publisher, whenever it joins |
| `'best_available'` | BEST_AVAILABLE reliability and durability (Iron and later) of the publishers discovered when the subscription is created; a publisher with a weaker QoS that joins later is not read. Falls back to `'any'` on Humble |
| a `QoSProfile` (e.g. `rclpy.qos.qos_profile_sensor_data`) | as given |
| an `int` | that depth, rclpy's default (reliable) QoS |

So by default a subscriber receives from both reliable and best-effort
publishers, but delivery is best effort. Pass `qos=` (e.g. a depth for
reliable delivery) where lost messages matter, and `latch=True` for the
last message of a `create_publisher(..., latch=True)` topic.

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

Or, to keep the original rospy code untouched for its ROS1 users, add a
jsk_ros1_ros2_compat version next to it and choose by whether
jsk_ros1_ros2_compat can be imported (not by `ROS_VERSION`):

```python
try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    ros_compat = None  # jsk_common < 3.0 (ROS1 only): the original code below

class Foo(object):
    def run(self):  # the original rospy code, unchanged
        rospy.init_node('foo')
        ...

    def run_compat(self, node):  # the same with jsk_ros1_ros2_compat (ROS1/ROS2)
        ...

if __name__ == '__main__':
    if ros_compat is None:
        Foo().run()
    else:
        Foo().run_compat(ros_compat.init_node('foo'))
```

### Python: `jsk_ros1_ros2_compat.rosbag`

One `rosbag.Bag`-like interface for ROS1 `.bag` files (`rosbag`) and ROS2
rosbag2 directories (`rosbag2_py`, `sqlite3` storage only), used by
jsk_rosbag_tools:

```python
from jsk_ros1_ros2_compat.rosbag import open_bag, nanoseconds_from_sec

with open_bag('in', 'r') as inbag, open_bag('out', 'w') as outbag:
    for topic, msg, t in inbag.read_messages(topics=['/tf']):
        outbag.write(topic, msg, t + nanoseconds_from_sec(1.0))
```

- `open_bag(path, mode='r')`: a reader with `read_messages(topics=None)`,
  `read_raw_messages()`/`deserialize_raw()`, `get_message_count()` and
  iteration, or (`'w'`) a writer with `write(topic, msg, t)` and
  `write_raw()`. Both are context managers.
- Bag timestamps are always integer nanoseconds on both versions; convert
  with `nanoseconds_from_sec()`, `stamp_from_nanoseconds()`,
  `nanoseconds_from_stamp()` and `stamp_to_sec()`.
- `get_topic_dict(path)` (ROS2 only): `{topic: {'type': 'pkg/Type', 'messages': n}}`.

### Python: `jsk_ros1_ros2_compat.log_utils`

`jsk_logdebug/info/warn/err/fatal(msg)` (with the node name and caller
prefixed) and `log*_throttle(period, msg)`. On ROS2 call
`log_utils.set_node(node)` once first; it is a no-op on ROS1.

## C++: `#include <jsk_ros1_ros2_compat/compat.h>`

| Header | Provides |
|---|---|
| `ros_version.h` | ROS1/ROS2 detection: `#if ROS_VERSION_MAJOR != 1` means ROS2 |
| `logging.h` | ROS2: `ROS_DEBUG`/`ROS_INFO`/`ROS_WARN`/`ROS_ERROR` (and `*_STREAM`) and `ros::param::param<T>("~name", value, default)` (declares a parameter) on `ROS1_ROS2_COMPAT::g_node`, which `main()` sets once |
| `time.h` | `ROS1_ROS2_COMPAT::RosTime`/`RosDuration`, `rosTimeNow()`, `stampMsg()`, `durationFromSec()`, `durationFromMsg()` |
| `pubsub.h` | `ROS1_ROS2_COMPAT::RosPublisher<T>`/`RosSubscriber<T>`/`RosServiceServer<T>`, `publishMsg()`, `topicName()`, `privateName()`, `createSubscriber<T>(nh, topic, queue_size, callback)` |
| `msg_alias.h` | `JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(pkg, Type)` (`pkg::Type` for `pkg::msg::Type`), `_SRV_ALIAS`, `_CONST_PTR_ALIAS` (`pkg::TypeConstPtr`), `_SRV_REQ_RES_ALIAS`; no-ops on ROS1 |
| `const_ptr.h` | `#define ConstPtr ConstSharedPtr` on ROS2; include it **after** all message headers (not included by `compat.h`) |

`compat.h` includes all of them except `const_ptr.h`. See the comments in
each header for details.

## CMake (ROS2)

Available after `find_package(jsk_ros1_ros2_compat REQUIRED)` (ament) or
`find_package(catkin REQUIRED COMPONENTS jsk_ros1_ros2_compat)` (catkin, where
only `jsk_ros1_ros2_compat_install_python()` is defined); add
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
