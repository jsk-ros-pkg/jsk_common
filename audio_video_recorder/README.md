# audio_video_recorder

ROS package for recording audio and video synchronously

![](.media/pr2_sample.gif)

[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)

## ROS 1 / ROS 2

The package builds for both ROS 1 (catkin) and ROS 2 (ament), selected by
`$ROS_VERSION`.

| | ROS 1 | ROS 2 |
|---|---|---|
| `audio_video_recorder` executable | yes | yes |
| `launch/audio_video_recorder.launch` | yes | no |
| `sample/sample_audio_video_recorder.launch`, `sample/pr2_audio_video_recorder.launch` | yes | no |

The launch files are installed on ROS 2 too, but they are ROS 1
(roslaunch XML) files and there are no ROS 2 launch files. On ROS 2 run the
executable directly. The node is named `audio_video_recorder`, so its private
topics are `/audio_video_recorder/input/image` and
`/audio_video_recorder/input/audio`:

```bash
ros2 run audio_video_recorder audio_video_recorder --ros-args \
  -r /audio_video_recorder/input/image:=/image_raw \
  -r /audio_video_recorder/input/audio:=/audio/audio \
  -p file_name:=/tmp/test.avi -p video_encoding:=RGB
```

## Sample

You can record audio and video on your laptop (ROS 1, uses `usb_cam` and
`audio_capture`).

```bash
roslaunch audio_video_recorder sample_audio_video_recorder.launch
```

`sample/pr2_audio_video_recorder.launch` records `/audio` and
`/kinect_head/rgb/image_rect_color` of a PR2.

## Subscribing Topics

- `~input/image` (`sensor_msgs/Image`)

  Video frames. The raw image data is pushed to GStreamer as is, so the image
  size and encoding must match `video_width`, `video_height` and
  `video_encoding`.

- `~input/audio` (`audio_common_msgs/AudioData`)

  Audio data, in the format given by the audio parameters.

## Parameters

Node: `audio_video_recorder/audio_video_recorder`

On ROS 1 these are private (`~`) parameters, on ROS 2 node parameters.

### Common parameters

- `queue_size` (`Int`, default: `100`)

  Queue size

- `file_name` (`String`, default: `/tmp/test.avi`)

  Output file location

- `file_format` (`String`, default: `avi`)

  Output file format (Only `avi` is supported now.)

### Audio parameters

- `audio_format` (`String`, default: `mp3`)

  Audio format of `~input/audio` (`mp3` or `wave`)

- `audio_sample_format` (`String`, default: `S16LE`)

  Audio sample format

- `audio_channels` (`Int`, default: `1`)

  Number of audio channel

- `audio_depth` (`Int`, default: `16`)

  Audio depth

- `audio_sample_rate` (`Int`, default: `16000`)

  Audio sample rate

### Video parameters

- `video_encoding` (`String`, default: `RGB`)

  Video encoding for `gstreamer`

- `video_height` (`Int`, default: `480`)

  Video image height

- `video_width` (`Int`, default: `640`)

  Video image width

- `video_framerate` (`Int`, default: `30`)

  Video frame rate
