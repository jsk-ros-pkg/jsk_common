Setup environmental variables for ROS
=====================================

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