# app_notification_saver

Plugins and nodes to save notification to json file and pass it to `app_notifier`

## `app_manager` plugins

### `app_notification_saver/service_notification_saver`: General notification saver plugin

This plugin saves notification via service call.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `json_path` : JSON path

#### Sample plugin description

```yaml
plugins:
  - name: service_notification_saver
    type: app_notification_saver/service_notification_saver
    launch_args:
      json_path: /tmp/app_notification.json
```

#### Save app notification

You can save app notification with service call.

```bash
rosservice call /service_notification_saver/save_app_notification "title: 'object recognition'
stamp:
  secs: 1627467479
  nsecs: 13279914
location: 'kitchen'
message: 'Dish is found'"
```

#### Clear app notification

You can also clear app notification.

```bash
rosservice call /service_notification_saver/clear_app_notification "{}"
```

### `app_notification_saver/smach_notification_saver`: SMACH notification saver plugin

This plugin saves notification via service call.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `json_path` : JSON path
- `smach_status_topic`: SMACH status topic name

#### Sample plugin description

```yaml
plugins:
  - name: smach_notification_saver
    type: app_notification_saver/smach_notification_saver
    launch_args:
      json_path: /tmp/app_notification.json
      smach_status_topic: /server_name/smach/container_status
```

## Nodes

### `service_notification_saver_node.py`: Node for general notification saver

Save notification node via service call.

#### Services

- `~save_app_notification` (`app_notification_saver/SaveAppNotification`)

  Service to save app notification to JSON.

- `~clear_app_notification` (`std_srvs/Empty`)

  Service to clear app notification in JSON.

#### Parameters

- `~json_path` (`String`, default: `/tmp/app_notification.json`)

  Path to json file which contains app notification

#### Sample

##### Launch service_notification_saver node

```bash
roslaunch app_notification_saver service_notification_saver.launch
```

##### Save app notification

You can save app notification with service call.

```bash
rosservice call /service_notification_saver/save_app_notification "title: 'object recognition'
stamp:
  secs: 1627467479
  nsecs: 13279914
location: 'kitchen'
message: 'Dish is found'"
```

##### Clear app notification

You can also clear app notification.

```bash
rosservice call /service_notification_saver/clear_app_notification "{}"
```

##### Check output JSON

The sample output of the json file is like below:

```json
{
    "object recognition": [
        {
            "date": "2021-07-28T19:17:59",
            "message": "Dish is found",
            "location": "kitchen"
        },
        {
            "date": "2021-07-28T19:18:09",
            "message": "Cup is found",
            "location": "kitchen"
        }
    ],
    "navigation failure": [
        {
            "date": "2021-07-28T19:18:29",
            "message": "Stucked in front of the chair",
            "location": "living room"
        }
    ]
}
```

### `smach_notification_saver_node.py`: Node for SMACH notification saver

Save notification of smach state.

#### Subscribe topics

- `~smach/container_status` (`smach_msgs/SmachContainerStatus`, default: `/server_name/smach/container_status`)

  Smach status topic

#### Parameters

- `~json_path` (`String`, default: `/tmp/app_notification.json`)

  Path to json file which contains app notification

#### Sample

##### Launch smach_notification_saver node

```bash
# Launch only smach_notification_saver node
roslaunch app_notification_saver smach_notification_saver.launch

# Sample
# Launch smach_notification_saver node and rosbag
roslaunch app_notification_saver sample_smach_notification_saver.launch --screen
```
