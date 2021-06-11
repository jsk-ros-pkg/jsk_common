# Relay

`jsk_topic_tools/Relay` is a node/nodelet that subscribes to `~input` topic and
republishes all incoming data to `~output` topic like `topic_tools/Relay` but
has several additional features such as latch and lazy mode.

## Subscribing Topics

* `~input` (`AnyMsg`)

  Incoming topic to be relayed

## Publishing Topics

* `~output` (`AnyMsg`, same type as `~input`)

  Outgoing topic to publish on

## Parameters

* `~always_subscribe` (`bool`, default: `false`)

    If `true`, it turns off its lazy mode, where the input topic is subscribed
    only when the output topic is subscribed.

* `~latch` (`bool`, default: `false`)

    If `true`, it publishes the output topic in latch mode.
