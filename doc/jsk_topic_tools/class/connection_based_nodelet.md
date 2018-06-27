# ConnectionBasedNodelet (C++)

**WARNING**

This base-class is being deprecated and replaced by `nodelet_topic_tools::NodeletLazy` in
[nodelet_topic_tools](http://wiki.ros.org/nodelet_topic_tools).

## Description

This class is a base-class which can start subscribing topics if published topics are subscribed by the other node.
This is abstruct class.

## Note

Each subclass of this class must call `onInitPostProcess` in the last line `onInit`.

## Parameter
- `~use_multithread_callback` (Bool, default: `true`):

If true, node use getMTNodeHandle and getMTPrivateNodeHandle for getting NodeHandle.
MTNodeHandle generates threads for processing its callback function.

If false, node use getNodeHandle and getPrivateNodeHandle.
This is default behavior of normal node (not nodelet).

Please see [nodelet](http://wiki.ros.org/nodelet)

- `~always_subscribe` (Bool, default: `false`):

  Subscribes topics even if there is no subscribers of advertised topics if `true`.
- `~verbose_connection` (Bool, default: `false`):

  Show verbose log message on topic connection

- `~no_warn_on_init_post_process` (Bool, default: `false`):

  Never warn if `onInitPostProcess` method is not yet called.
