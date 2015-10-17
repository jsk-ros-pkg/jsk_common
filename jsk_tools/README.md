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

rosbag\_record\_interactive
---------------------------
You can choose topics to record in GUI and record them into a bag file

restart\_travis
---------------
Restart a travis job for specified github repository.
Firstly, add below in your `.bashrc`:

```sh
# you can get SLACK_TOKEN at https://api.slack.com/web.
export SLACK_TOKEN=xoxp-XXXXXXXX-XXXXXXXX-XXXXXXXX
```

To restart travis, need the repository slug (ex: `jsk-ros-pkg/jsk_common`) and job id (ex: `2019.6`):

```sh
$ restart_travis jsk-ros-pkg/jsk_common 2019.6  # usage: restart_travis <repo_slug> <job_id>
sending... 'restart travis jsk-ros-pkg/jsk_common 2019.6' -> #travis
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

```
usage: bag_plotter.py [-h] [--duration DURATION] [--start-time START_TIME]
                      config bag

Plot from bag file

positional arguments:
  config                yaml file to configure plot
  bag                   bag file to plot

optional arguments:
  -h, --help            show this help message and exit
  --duration DURATION, -d DURATION
                        Duration to plot
  --start-time START_TIME, -s START_TIME
                        Start timestamp to plot
```

Format of yaml file is like:
```yaml
global:
  layout: <vertical, horizontal or manual, optional and defaults to vertical>
plots:
  - title:  <title of plot, required>
    topic:  <list of topics, required>
    field:  <list of accessor to get value to plot, required>
    layout: <2-d layout. if global/layout is manual, this field is required>
    legend: <show legend or not, optional and defaults to true>
```

the length of topic and field should be same.
You can use "array format" in field.

* `a/b/c[0]` means the 1st element of array `a/b/c`.
* `a/b/c[0:3]` means 1st, 2nd and 4rd elements of array `a/b/c`.

Example is like:
```yaml
global:
  layout: vertical
plots:
  - title: "rleg temp"
    topic: [/motor_states]
    field: ["driver_temp[0:6]"]
  - title: "lleg temp"
    topic: [/motor_states]
    field: ["driver_temp[6:12]"]
  - title: "leg force (z)"
    topic: [/off_lfsensor, /off_rfsensor]
    field: [wrench/force/z, wrench/force/z,]
```

When you want to use manual layout, example should be like follows:
```
global:
  layout: manual
plots:
  - title: "rleg temp"
    topic: [/motor_states]
    field: ["driver_temp[0:6]"]
    layout: [0, 0]
  - title: "lleg temp"
    topic: [/motor_states]
    field: ["driver_temp[6:12]"]
    layout: [0, 1]
  - title: "chest temp"
    topic: [/motor_states]
    field: ["driver_temp[12:15]"]
    layout: [1, 0]
  - title: "head temp"
    topic: [/motor_states]
    field: ["driver_temp[15:17]"]
    layout: [1, 1]
```

roscore_regardless.py
---------------------
This script always checks roscore liveness and automatically run and kill a program.

```
rosrun jsk_tools roscore_regardless.py rostopic echo /foo
```


emacs
-----
We strongly recommend to use `dot.emacs` for common user.

```lisp
(load "~/ros/hydro/src/jsk-ros-pkg/jsk_common/jsk_tools/dot-files/dot.emacs")
```


### inferior-lisp-mode
![](images/inferior-lisp.gif)

inferior-lisp-mode is a classic lisp environment on emacs.
You can invoke roseus by `C-c e`.

When you use inferior-lisp-mode, you can immediately evaluate sexpression
without copy-and-paste it by `C-x C-e`.

tmux
----
Write following line in your `~/.tmux.conf`

```
source-file ~/ros/hydro/src/jsk-ros-pkg/jsk_common/jsk_tools/dot-files/tmux.conf
```
