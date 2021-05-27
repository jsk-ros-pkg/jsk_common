# audio_video_recorder

ROS package for recording audio and video synchronously

![[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)
](.media/pr2_sample.gif)

[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)

## Sample launch

You can record audio and video in one file (i.e. `AVI` file) on your computer.

```bash
roslaunch audio_video_recorder sample_audio_video_recorder.launch
```

## Node: `audio_video_recorder`

Node for recording audio and image topics into video file (i.e. `AVI` file)

### Subscribing topics

- `~input/audio` (`audio_common_msgs/AudioData`)

  Audio topic name in `audio_common_msgs/AudioData` format

- `~input/image` (`sensor_msgs/Image`)

  Image topic name in `sensor_msgs/Image` format

### Parameters

#### Common parameters for recording

- `file_name` (`String`, default: `/tmp/test.avi`)

  Output file name

- `file_format` (`String`, default: `avi`)

  Output file format (Only `avi` is supported now.)


#### Common parameters for subscribing topic

- `queue_size` (`Int`, default: `100`)

  Queue size

#### Parameters for subscribing audio topic `audio_common_msgs/AudioData`

Audio topic should be `audio_common_msgs/AudioData` format.

- `audio_format` (`String`, default: `mp3`)

  Format of audio topic. `mp3` and `wave` are supported.

- `audio_sample_format` (`String`, default: `S16LE`)

  Sample format of audio topic.

- `audio_channels` (`Int`, default: `1`)

  Number of channel of audio topic.

- `audio_depth` (`Int`, default: `16`)

  Depth of audio topic

- `audio_sample_rate` (`Int`, default: `16000`)

  Sample rate of audio topic

#### Parameters for subscribing image topic `sensor_msgs/Image`

Image topic should be `sensor_msgs/Image` format

- `video_encoding` (`String`, default: `RGB`)

  Encoding of image topic. `RGB` and `BGR` is supported.

- `video_height` (`Int`, default: `480`)

  Height of image topic.

- `video_width` (`Int`, default: `640`)

  Width of image topic

- `video_framerate` (`Int`, default: `30`)

  Frame rate of image topic
