# rostopic_connection_list

## Description

Tool to check rostopic connection with different hosts.

## Usage

```bash
$ rosrun jsk_topic_tools rostopic_connection_list --help
usage: rostopic_connection_list [-h] [--subscriber-host SUBSCRIBER_HOST]
                                [--publisher-host PUBLISHER_HOST]
                                [--publisher-host-sort] [--include-rosout]
                                [--show-nodes]

optional arguments:
  -h, --help            show this help message and exit
  --subscriber-host SUBSCRIBER_HOST, -s SUBSCRIBER_HOST
                        Subscriber hostname: (default: all)
  --publisher-host PUBLISHER_HOST, -p PUBLISHER_HOST
                        Publisher hostname: (default: all)
  --publisher-host-sort
                        Sort by publisher host name or not (default: False)
  --include-rosout      Include /rosout topic or not: (default: False)
  --show-nodes          Show node names or not: (default: False)
```

## Example

### Show all connection

```bash
$ rosrun jsk_topic_tools rostopic_connection_list
Subscriber host: 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/transmit -> 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/receive -> 133.11.216.168
    133.11.216.211 -> /tf -> 133.11.216.168
    133.11.216.211 -> /joint_states -> 133.11.216.168
    133.11.216.211 -> /tf_static -> 133.11.216.168
    pr1040 -> /tf -> 133.11.216.168
    pr1040s -> /audio -> 133.11.216.168
    pr1040s -> /battery/server2 -> 133.11.216.168
    pr1040s -> /kinect_head/rgb/image_rect_color/compressed -> 133.11.216.168
    pr1040s -> /tf -> 133.11.216.168
Subscriber host: 133.11.216.211
    pr1040 -> /l_arm_controller/follow_joint_trajectory/goal -> 133.11.216.211
    pr1040 -> /l_gripper_sensor_controller/event_detector -> 133.11.216.211
    pr1040 -> /robotsound_jp/cancel -> 133.11.216.211
...
...
(long lines)
```

### Specify subscriber and publisher host name

You can specify subscriber host and publisher host by `-s` and `-p`.

```bash
$ rosrun jsk_topic_tools rostopic_connection_list -s 133.11.216.168 -p 133.11.216.211
Subscriber host: 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/transmit -> 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/receive -> 133.11.216.168
    133.11.216.211 -> /tf -> 133.11.216.168
    133.11.216.211 -> /joint_states -> 133.11.216.168
    133.11.216.211 -> /tf_static -> 133.11.216.168
```

### Sort by publisher host name

You can sort by publisher host name with `--publisher-host-sort`.

```bash
$ rosrun jsk_topic_tools rostopic_connection_list --publisher-host-sort
Publisher host: 133.11.216.211
    133.11.216.211 -> /pr1040/wan0/transmit -> 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/receive -> 133.11.216.168
    133.11.216.211 -> /tf -> 133.11.216.168
    133.11.216.211 -> /joint_states -> 133.11.216.168
    133.11.216.211 -> /tf_static -> 133.11.216.168
...
...
(long lines)
```

### Show nodes

`--show-nodes` shows all node names, too.

```bash
$ rosrun jsk_topic_tools rostopic_connection_list --show-nodes
Subscriber host: 133.11.216.168
    133.11.216.211 -> /pr1040/wan0/transmit -> 133.11.216.168
        Publisher nodes:
            /network_status
        Subscriber nodes:
            /network_states_logger
    133.11.216.211 -> /pr1040/wan0/receive -> 133.11.216.168
        Publisher nodes:
            /network_status
        Subscriber nodes:
            /network_states_logger
    133.11.216.211 -> /tf -> 133.11.216.168
        Publisher nodes:
            /robot_state_publisher
            /robot_pose_ekf
            /realtime_loop
        Subscriber nodes:
            /base_transform_logger
            /map_transform_logger
...
...
(long lines)
```
