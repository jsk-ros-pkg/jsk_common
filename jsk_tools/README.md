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
