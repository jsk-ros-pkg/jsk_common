sanity\_lib.py
==============


## check Topic is published

- If you set `echo` param as True, the topic message will be shown in terminal

**Example**
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

**Example**
```
from jsk_tools.sanity_lib import *
rospy.init_node("check_sanity", anonymous = True)
checkNodeState("/listener", True)
```

## check Params
**Example**
```
from jsk_tools.sanity_lib import *
rospy.init_node("check_sanity", anonymous = True)
checkROSParam("/param_test", 5)
```
