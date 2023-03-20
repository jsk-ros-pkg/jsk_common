# app_recorder

Recorder plugin for `app_manager`

## `app_manager` plugins

### `app_recorder/audio_video_recorder_plugin`: Audio-video recorder plugin

This plugin records video data with audio during when an app is running.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `video_path`:
  - video file directory path
- `video_title`:
  - video file name
- `audio_topic_name`:
  - image topic name for audio
- `audio_channels`:
  - audio channels
- `audio_sample_rate`:
  - audio sample rate
- `audio_format`:
  - audio format
- `audio_sample_format`:
  - audio sample format
- `video_topic_name`:
  - image topic name for video
- `video_height`:
  - video height
- `video_width`:
  - video width
- `video_framerate`:
  - video framerate
- `video_encoding`:
  - video encoding
- `use_comrpressed`: (default: `False`)
  - Use compressed image topic or not
- `video_decompressed_topic_name`:
  - decompressed image topic name when `use_comrpressed` is `True`
- `use_machine`: (default: `False`)
  - Use machine tag or not
- `machine_name`:
  - machine name when `use_machine` is `True`
- `machine_file`:
  - machine file path when `use_machine` is `True`

#### Sample plugin description

```yaml
plugins:
  - name: audio_video_recorder_plugin
    type: app_recorder/audio_video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: test.avi
      audio_topic_name: /audio
      audio_channels: 1
      audio_sample_rate: 16000
      audio_format: wave
      audio_sample_format: S16LE
      video_topic_name: /head_camera/rgb/image_rect_color
      video_height: 480
      video_width: 640
      video_framerate: 30
      video_encoding: RGB
```

### `app_recorder/video_recorder_plugin`: Video recorder plugin

This plugin records video data during when an app is running.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `video_path`:
  - video file directory path
- `video_title`:
  - video file name
- `video_topic_name`:
  - image topic name for video
- `video_fps`:
  - video fps
- `video_codec`: (default: `XVID`)
  - video codec
- `use_comrpressed`: (default: `False`)
  - Use compressed image topic or not
- `video_decompressed_topic_name`:
  - decompressed image topic name when `use_comrpressed` is `True`
- `use_machine`: (default: `False`)
  - Use machine tag or not
- `machine_name`:
  - machine name when `use_machine` is `True`
- `machine_file`:
  - machine file path when `use_machine` is `True`

#### Sample plugin description

```yaml
plugins:
  - name: video_recorder_plugin
    type: app_recorder/video_recorder_plugin
    launch_args:
      video_path: /tmp
      video_title: test.avi
      video_topic_name: /wide_stereo/right/image_rect_color
      video_fps: 30
```

### `app_recorder/audio_recorder_plugin`: Audio recorder plugin

This plugin records audio data during when an app is running.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `audio_path`:
  - audio file directory path
- `audio_title`:
  - audio file name
- `audio_topic_name`:
  - image topic name for audio
- `audio_format`: (default: `wave`)
  - audio format
- `use_machine`: (default: `False`)
  - Use machine tag or not
- `machine_name`:
  - machine name when `use_machine` is `True`
- `machine_file`:
  - machine file path when `use_machine` is `True`

#### Sample plugin description

```yaml
plugins:
  - name: audio_recorder_plugin
    type: app_recorder/audio_recorder_plugin
    launch_args:
      audio_path: /tmp
      audio_title: test.mp3
      audio_topic_name: /audio
      audio_format: mp3
```

### `app_recorder/rosbag_recorder_plugin`: Rosbag recorder plugin

This plugin records rosbag data during when an app is running.

#### `plugin_args`: Plugin arguments

`None`

#### `launch_args`: Plugin launch arguments

- `rosbag_path`:
  - rosbag file directory path
- `rosbag_title`:
  - rosbag file name
- `rosbag_topic_names`:
  - topic names for rosbag
- `compress`: (default: `False`)
  - compress rosbag or not
- `use_machine`: (default: `False`)
  - Use machine tag or not
- `machine_name`:
  - machine name when `use_machine` is `True`
- `machine_file`:
  - machine file path when `use_machine` is `True`

#### Sample plugin description

```yaml
plugins:
  - name: rosbag_recorder_plugin
    type: app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: test.bag
      rosbag_topic_names: /tf /joint_states
```

### `app_recorder/result_recorder_plugin`: Result recorder plugin

This plugin records app result in yaml when app finishs.

#### `plugin_args`: Plugin arguments

- `result_path`: result file directory path
- `result_title`: result file name

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: result_recorder_plugin
    type: app_recorder/result_recorder_plugin
    plugin_args:
      result_path: /tmp
      result_title: test.yaml
```

### `app_recorder/rosbag_audio_converter_plugin`: Rosbag audio converter plugin

This plugin converts rosbag to audio file when app finishs.

#### `plugin_args`: Plugin arguments

- `rosbag_path`:
  - rosbag file directory path
- `rosbag_title`:
  - rosbag file name
- `audio_path`:
  - Output audio file (.wav)
- `audio_topic_name`:
  - Input audio topic name
- `audio_sample_rate`:
  - Input audio sample rate
- `audio_channels`:
  - Input audio the number of channels

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: respeaker_audio_converter_plugin
    type: app_recorder/rosbag_audio_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      audio_path: /tmp/go_to_kitchen_audio.wav
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
```

### `app_recorder/rosbag_video_converter_plugin`: Rosbag video converter plugin

This plugin converts rosbag to video file when app finishs. If `audio_topic_name` is given, video with audio is generated.

#### `plugin_args`: Plugin arguments

- `rosbag_path`:
  - rosbag file directory path
- `rosbag_title`:
  - rosbag file name
- `video_path`:
  - Output video file (.mp4)
- `image_topic_name`:
  - Input image topic name
- `image_fps`:
  - Input image frame rate
- `audio_topic_name`:
  - Input audio topic name
- `audio_sample_rate`:
  - Input audio sample rate
- `audio_channels`:
  - Input audio the number of channels

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: head_camera_converter_plugin
    type: app_recorder/rosbag_video_converter_plugin
    plugin_args:
      rosbag_path: /tmp
      rosbag_title: go_to_kitchen_rosbag.bag
      video_path: /tmp/go_to_kitchen_head_camera.mp4
      image_topic_name: /head_camera/rgb/throttled/image_rect_color/compressed
      image_fps: 5
      audio_topic_name: /audio
      audio_sample_rate: 16000
      audio_channels: 1
```
