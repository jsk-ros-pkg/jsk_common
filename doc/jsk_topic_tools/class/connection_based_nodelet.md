# connection_based_nodelet

## Description

This class is a base-class which can start subscribing topics if published topics are subscribed by the other node.
This is abstruct class.

## Parameter
- `~use_multithread_callback` (Bool, default: `true`):

If true, node use getMTNodeHandle and getMTPrivateNodeHandle for getting NodeHandle.
MTNodeHandle generates threads for processing its callback function.

If false, node use getNodeHandle and getPrivateNodeHandle.
This is default behavior of normal node (not nodelet).

Please see [nodelet](http://wiki.ros.org/nodelet)

- `~always_subscribe` (Bool, default: `false`):

- `~verbose_connection` (Bool, default: `false`):

