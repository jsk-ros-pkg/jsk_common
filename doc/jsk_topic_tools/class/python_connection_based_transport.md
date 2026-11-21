# ConnectionBasedTransport (Python)

## Description

This class is a base-class which can start subscribing topics if published topics are subscribed by the other node.
This is abstruct class.

Users inherit this class and define publisher by calling the `self.advertise` function.
The return value of `self.advertise` is the `Publisher` class. User should execute publish in `callback` function.

### For Expert

When `publish` is executed, it is judged that this node is running normally, and published `diagnostics` becomes `OK` level.
However, if the publisher is subscribed but the `publish` function is not called, the node is not working properly and `diagnostics` will be at the `ERROR` level.

Some users may execute`publish` at any time (eg `rospy.Timer`).
In that case, by executing the `self.poke` function in the callback function, the state of `diagnostics` will be `OK` level without executing publish.


## Parameter

* `~always_subscribe` (Bool, default: `false`):

  Subscribes topics even if there is no subscribers of advertised topics if `true`.

* `~enable_vital_check` (Bool, default: `true`):

  If this value is `true`, `/diagnostics` will be published and the status of whether the topic has been published will be output from this node.

* `~vital_rate` (Double, default: `1.0`):

  Rate to determine if the nodelet is in health.
  If the rate that the callback functions is below this parameter, error messages are displayed on diagnostics.
  This value is valid only if `~enable_vital_check` is `true`.

* `/diagnostic_nodelet/use_warn` or `~use_warn` (Bool, default: `False`):

  If this parameter is enabled, diagnostic messages on failure is displayed on `WARN` level instead of `ERROR` level.
  `/diagnostic_nodelet/use_warn` affects every nodelets that inherits this class, but it still can be overriden for each nodelet by setting `~use_warn` parameter.

## Publishing Topic

* `/diagnostics` (`diagnostic_msgs.DiagnosticArray`):

  Diagnostic messages. Required if `~enable_vital_check:=true`

## How does it behaves?

```bash
# terminal 1:
$ roslaunch jsk_topic_tools test_connection_based_transport.test

# terminal 2:
$ rostopic echo /simple_image_transport/output

# terminal 3:
$ rostopic info /simple_image_transport/output
Type: sensor_msgs/Image

Publishers:
 * /simple_image_transport (http://133.11.216.160:42481/)

Subscribers:
* /rostopic_137980_1496651422064 (http://133.11.216.160:38414/)

# terminal 2:
$ ^C  # cancel

# terminal 3:
$ rostopic info /simple_image_transport/output
Type: sensor_msgs/Image

Publishers:
 * /simple_image_transport (http://133.11.216.160:42481/)

Subscribers: None
```

## How to use it?

See `jsk_topic_tool/sample/simple_image_transport.py` to how to use it.

```python
import rospy

from sensor_msgs.msg import Image

from jsk_topic_tools import ConnectionBasedTransport


class SimpleImageTransport(ConnectionBasedTransport):
    def __init__(self):
        super(SimpleImageTransport, self).__init__()
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub_img = rospy.Subscriber('~input', Image, self._process)

    def unsubscribe(self):
        self.sub_img.unregister()

    def _process(self, img_msg):
        self._pub.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('sample_image_transport')
    img_trans = SimpleImageTransport()
    rospy.spin()
```

## Checking the node status by diagnostics

You can check diagnostics by setting `~enable_vital_check` to `true`.

```bash
# terminal 1:
$ roslaunch jsk_topic_tools sample_connection_based_transport.launch

# terminal 2:
$ rostopic list
/diagnostics
/diagnostics_agg
/diagnostics_toplevel_state
/input
/input_dummy
/input_image
/mux/selected
/rosout
/rosout_agg
/simple_image_transport/output

# terminal 3:
$ rosrun rqt_robot_monitor rqt_robot_monitor
```

There is no one to subscribe to `/simple_image_transport/output`, so diagnostics is OK.

![](images/diagnostics_no_subscribers.jpg)

```bash
# terminal 4:
rostopic echo /simple_image_transport/output
```

If you subscribe to `/simple_image_transport/output`,
it will change to the diagnostics message `/simple_image_transport is running`.

![](images/diagnostics_is_running.jpg)


```bash
# terminal 5:
rosservice call /mux/select input_dummy
```

When changing input for `simple_image_transport` by mux,
the error message will change to `/simple_image_transport is not running`.

![](images/diagnostics_is_not_running.jpg)

You can check if the node is running correctly like these.

## ROS 2

On ROS 2, `ConnectionBasedTransport` is an `rclpy.lifecycle.LifecycleNode`, like the C++
`jsk_topic_tools::ConnectionBasedLifecycleNode`, so the processing state of the node is in the lifecycle system:

* it is `inactive` (configured) when created;
* when an advertised topic gets its first subscriber, it is activated (its publishers, which are lifecycle publishers, are
  enabled) and `subscribe()` is called;
* when none of the advertised topics has any subscriber, `unsubscribe()` is called and it is deactivated;
* `~always_subscribe` activates it and calls `subscribe()` at once; `cleanup` and `shutdown` call `unsubscribe()`.

The subscribers are told by the matched events of the publishers (Iron and later). `is_active()` returns whether it is active.

The differences from ROS 1:

* The transport is the node itself: the constructor takes the node name (and the keyword arguments of
  `rclpy.node.Node`, e.g. `parameter_overrides`), and `self.node` is the transport itself, to be given to
  `jsk_ros1_ros2_compat.rospy_rclpy_compat`. On ROS 1, the constructor takes the node name too but ignores it
  (`rospy.init_node()` names the node), and `self.node` is `None`.
* Parameters: `~always_subscribe` and `~verbose_connection` (Bool, default: `false`, logs the connections and the
  subscription changes). The vital check (`~enable_vital_check`, `~vital_rate`, `~use_warn`, `/diagnostics`) and
  `--inout` are of ROS 1 only; `poke()` does nothing.

A subclass for both ROS 1 and ROS 2:

```python
from sensor_msgs.msg import Image

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros
from jsk_topic_tools import ConnectionBasedTransport


class SimpleImageTransport(ConnectionBasedTransport):
    def __init__(self, node_name='simple_image_transport'):
        super(SimpleImageTransport, self).__init__(node_name)
        self._pub = self.advertise('~output', Image, queue_size=1)

    def subscribe(self):
        self.sub_img = ros.create_subscription(
            self.node, '~input', Image, self._process, queue_size=1)

    def unsubscribe(self):
        if ros.ROS_VERSION == 1:
            self.sub_img.unregister()
        else:
            self.destroy_subscription(self.sub_img)

    def _process(self, img_msg):
        self._pub.publish(img_msg)


if __name__ == '__main__':
    if ros.ROS_VERSION == 1:
        ros.init_node('simple_image_transport')
        img_trans = SimpleImageTransport()
        ros.spin(None)
    else:
        ros.rclpy.init()
        img_trans = SimpleImageTransport()
        ros.spin(img_trans)
```

How it behaves on ROS 2:

```bash
# terminal 1:
$ python3 simple_image_transport.py

# terminal 2:
$ ros2 lifecycle get /simple_image_transport
inactive [2]

# terminal 3:
$ ros2 topic echo /simple_image_transport/output

# terminal 2:
$ ros2 lifecycle get /simple_image_transport
active [3]
$ ros2 topic info /simple_image_transport/input
Type: sensor_msgs/msg/Image
Publisher count: 0
Subscription count: 1

# terminal 3:
$ ^C  # cancel

# terminal 2:
$ ros2 lifecycle get /simple_image_transport
inactive [2]
$ ros2 topic info /simple_image_transport/input
Unknown topic '/simple_image_transport/input'
```
