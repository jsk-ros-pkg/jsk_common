# constant_rate_throttle_nodelet

![images/lightweight_throttle_nodelet_diagram.svg](images/lightweight_throttle_nodelet_diagram.svg)

## Description

This nodelet provides function like `lightweight_throttle_nodelet`, but support accurate publish rate.

The rate of throttled message is configurable by passing `~update rate` parameter on launching this nodelet.

## Subscribing Topic
- `~input` (`AnyMsg`):

  Input topic. This topic is throttled to low publish rate.

## Publishing Topic
- `~output` (`AnyMsg`):

  Throttled topic.
  Publish rate of throttled topic is configurable by setting `~update_rate` parameter.
  If `~update_rate` is higher than input topic rate, published message is used from buffer.

## Parameter
- `~update_rate` (Double, default: `1.0`):

  Publish rate of throttled message [Hz]. This parameter is updated only on launching this nodelet.
