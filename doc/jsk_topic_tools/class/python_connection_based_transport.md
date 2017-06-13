# ConnectionBasedTransport (Python)

**WARNING**

This base-class is being deprecated and replaced by `topic_tools.LazyTransport` in
[topic_tools](http://wiki.ros.org/topic_tools).

## Description

This class is a base-class which can start subscribing topics if published topics are subscribed by the other node.
This is abstruct class.

## Parameter

* `~always_subscribe` (Bool, default: `false`):

  Subscribes topics even if there is no subscribers of advertised topics if `true`.

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
