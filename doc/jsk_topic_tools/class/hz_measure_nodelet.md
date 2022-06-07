# HzMeasureNodelet (C++)

## Description

This node publishes `hz` of target topic and `diagnostics`.

User can specify `~warning_hz` param.

If target `hz` is smaller than `~warning_hz`, this node outputs `diagnostics` at `WARN` level (if `~use_warn` is `True`) or `ERROR` level.


## Subscribing Topic

- `~input` (`AnyMsg`):

    Target topic.

## Publishing Topic

- `~output` (`std_msgs::Float32`):

    Target topic's `hz`.

- `~diagnostics` (`diagnostic_msgs::DiagnosticArray`):

    Diagnostic messages.

## Parameter
- `~message_num` (`Int`, default: `10`):

    Calculate `hz` from the arrival times of `~message_num` topics.

- `~warning_hz` (`Double`, default: `-1`):

    If target `hz` is smaller than `~warning_hz`, this node outputs `diagnostics` at `WARN` level (if `~use_warn` is `True`) or `ERROR` level.

- `~use_warn` (Bool, default: `False`):

    If this parameter is enabled, diagnostic messages on failure is displayed on `WARN` level instead of `ERROR` level.
