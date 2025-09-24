# app_uploader

Uploader plugin for `app_manager`

## `app_manager` plugins

### `app_uploader/gdrive_uploader_plugin`: Google drive uploader plugin

This plugin depends on [gdrive_ros](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros).

#### `plugin_args`: Plugin arguments

- `upload_file_paths`: upload file directory paths
- `upload_file_titles`: upload file names
- `upload_parents_path`: google drive upload path
- `upload_server_name`: `gdrive_ros/gdriver_serve_node` server name

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: gdrive_uploader_plugin
    type: app_uploader/gdrive_uploader_plugin
    plugin_args:
      upload_file_paths:
        - /tmp/test.avi
        - /tmp/test.bag
      upload_file_titles:
        - test.avi
        - test.bag
      upload_parents_path: logs
      upload_server_name: /gdrive_server
```
