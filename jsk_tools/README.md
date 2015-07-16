jsk_tools
=========

You can use commands below after `source /opt/ros/$DISTRO/setup.bash`.

rossetip
--------
Setup your `ROS_IP` and `ROS_HOSTNAME`.

```sh
$ rossetip
set ROS_IP and ROS_HOSTNAME to 192.168.11.1
$ echo $ROS_IP, $ROS_HOSTNAME
192.168.11.1, 192.168.11.1
```


rossetlocal
-----------
Setup your `ROS_MASTER_URI` to localhost.

```sh
$ rossetlocal
set ROS_MASTER_URI to http://localhost:11311
$ echo $ROS_MASTER_URI
http://localhost:11311
```


rossetmaster
------------
Setup your `ROS_MASTER_URI` to robot's hostname.

```sh
# rossetmaster ${hostname} ${ros_port}
# default: hostname=pr1040, ros_port=11311
user@host $ rossetmaster
set ROS_MASTER_URI to http://pr1040:11311
[http://pr1040:11311] user@host $ echo $ROS_MASTER_URI
http://pr1040:11311
```


rosdefault
----------
Setup `ROS_MASTER_URI` with default hostname written in `~/.rosdefault`.

```sh
$ cat ~/.rosdefault
pr1040
$ rosdefault
set ROS_MASTER_URI to http://pr1040:11311
```

It is recommended to run `rosdefault` in your .bashrc or .zshrc.


rossetdefault
-------------
Setup your default hostname.
After running this command, you can setup `ROS_MASTER_URI` with default hostname by `rosdefault`.
(default hostname will be stored at `~/.rosdefault`)

```sh
# rossetdefault ${hostname}
# default: hostname=local
$ rossetdefault baxter
set ROS_MASTER_URI to http://baxter:11311
$ bash
$ rosdefault
set ROS_MASTER_URI to http://baxter:11311
```


# sanity_lib.py
## check Topic is published

- If you set `echo` param as True, the topic message will be shown in terminal

### Example
```
from jsk_tools.sanity_lib import *
from std_msgs.msg import String
rospy.init_node("check_sanity", anonymous = True)
checkTopicIsPublished("/chatter", String)
```
## check Node State
There is 4 cases
- Node exists, and you want to exist.
- Node exists, and you don't want to exist
- Node doesn't exist and you want to exist
- Node doesn't exist and you don't want to exist

The second parameter is Needed Parameter.
### Example
```
from jsk_tools.sanity_lib import *
rospy.init_node("check_sanity", anonymous = True)
checkNodeState("/listener", True)
```
## check Params
### Example
```
from jsk_tools.sanity_lib import *
rospy.init_node("check_sanity", anonymous = True)
checkROSParam("/param_test", 5)
```

bag_plotter.py
--------------
bag_plotter is a script to plot from a bag file directly.
![](images/bag_plotter.png)


Usage is
```
$ bag_plotter.py bag_file conf_yaml
```

Format of yaml file is like:
```yaml
global:
  layout: "vertical"
plots:
  - title: "pose_x and pose_y"
    type: "line"
    topic: ["/Board/pose", "/Board/pose"]
    field: ["pose/position/x", "pose/position/y"]
  - title: "pose_x"
    type: "line"
    topic: ["/Board/pose"]
    field: ["pose/position/x"]
  - title: "pose_y"
    type: "line"
    topic: ["/Board/pose"]
    field: ["pose/position/y"]
```
