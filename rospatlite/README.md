rospatlite
==========

This is a ROS driver for patlite.

Usage
-----

Run patlite_node.py with following ros params.

- host : a hostname or ip address of your patlite
- port : a port for TCP connection of your patlite
- timeout : timeout of socket

Then you can see some topics under the private namespace.

    $ rostopic list
    /patlite/set/blue
    /patlite/set/buzzer
    /patlite/set/green
    /patlite/set/red
    /patlite/set/white
    /patlite/set/yellow
    $ rostopic pub /patlite/set/buzzer std_msgs/Int8 1 # <- buzzer ON
    $ rostopic pub /patlite/set/buzzer std_msgs/Int8 0 # <- buzzer OFF
    $ rostopic pub /patlite/set/red std_msgs/Int8 1 # <- red light ON
    $ rostopic pub /patlite/set/yellow std_msgs/Int8 2 # <- yellow light blink
