# audio_video_recorder

ROS package for recording audio and video synchronously

![[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)
](.media/pr2_sample.gif)

[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)

## Sample

You can record audio and video in one file (i.e. `AVI` file) on your computer.

```bash
roslaunch audio_video_recorder sample_audio_video_recorder.launch
```

## Parameters

Node: `audio_video_recorder/audio_video_recorder`

### Common parameters

- `queue_size` (`Int`, default: `100`)

  Queue size

- `file_name` (`String`, default: `/tmp/test.avi`)

  Output file name

- `file_format` (`String`, default: `avi`)

  Output file format (Only `avi` is supported now.)

### Audio parameters

Parameters for subscribing audio topic

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

### Video parameters

Parameters for subscribing image topic

- `video_encoding` (`String`, default: `RGB`)

  Encoding of image topic. `RGB` and `BGR` is supported.

- `video_height` (`Int`, default: `480`)

  Height of image topic.

- `video_width` (`Int`, default: `640`)

  Width of image topic

- `video_framerate` (`Int`, default: `30`)

  Frame rate of image topic
