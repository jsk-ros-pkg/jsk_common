# Passthrough

`jsk_topic_tools/Passthrough` is a node/nodelet to relay topics only for specified duration. You can use the service call to turn relay on and off. By default, the topic relay is turned off.

## Subscribing Topics

* `~input` (`AnyMsg`)

  Incoming topic to be relayed

## Publishing Topics

* `~output` (`AnyMsg`, same type as `~input`)

  Outgoing topic to publish on

## Services

* `~request` (`std_srvs/Empty`)

  Start topic relay

* `~stop` (`std_srvs/Empty`)

  Stop topic relay

* `~request_duration` (`jsk_topic_tools/PassthroughDuration`)

  Perform topic relay for a specified duration. Duration 0 means infinite relay.

## Parameters

* `~default_duration` (`double`, default: `10.0`)

  Duration [s] to relay the topic. Duration 0 means infinite relay.

## Usage
```
# Terminal 1
$ roslaunch jsk_topic_tools passthrough_sample.launch

# Terminal 2
$ rostopic pub /input std_msgs/String "data: 'hello'" -r10

# Terminal 3
$ rostopic echo /input

# Terminal 4
$ rostopic echo /output

# Terminal 5
$ rosservice call /passthrough_sample/request
$ rosservice call /passthrough_sample/stop
```
