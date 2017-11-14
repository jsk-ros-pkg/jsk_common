# StealthRelay

`jsk_topic_tools/StealthRelay` is a node/nodelet that subscribes to `~input` topic and republishes all incoming data to `~output` topic like `topic_tools/Relay` but only if `~monitor_topic` topic is subscribed by any other nodes.

When `~input` and `~monitor_topic` topic name point to the same topic, it looks that this node subscribes `~input` topic only when other nodes also subscribes it, which is equivalent to subscribing without incrementing subscription counter.
`~monitor_topic` can be set by rosparam and is set to `~input` by default.

## Subscribing Topics

* `~input` (`AnyMsg`)

  Incoming topic to be relayed

## Publishing Topics

* `~output` (`AnyMsg`, same type as `~input`)

  Outgoing topic to publish on

## Parameters

* `~use_multithread_callback` (`bool`, default: `True`)

    If `true`, initialize `NodeHandle` with multi threading.

* `~queue_size` (`int`, default: `1`)

    Queue size for subscription of the incoming topic.

* `~enable_monitor` (`bool`, default: `True`)

    If enabled, monitoring feature is enabled, otherwise this nodelet behaves as same as `Relay` nodelet.

* `~monitor_topic` (`string`, default: `~input`)

    Topic name to monitor.

* `~monitor_rate` (`double`, default: `1.0`)

    Desired monitoring rate of subscribing messages of `~monitor_topic`.
