# audio_video_recorder

ROS package for recording audio and video synchronously

![[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)
](.media/pr2_sample.gif)

[Full video on Google Drive](https://drive.google.com/file/d/1TWnRKbOdq6jPza82eNhhjn56lQXRxWjl/view?usp=sharing)

## Sample

You can record audio and video on your laptop.

```bash
roslaunch audio_video_recorder sample_audio_video_recorder.launch
```

If you want to sedd ROS Interface Sample (Topic/Service) of audio_video_recorder, Please run audio_video_recorder_server demo.

```
roslaunch audio_video_recorder sample_audio_video_recorder_server.launch
```

And if you want to see an example of client script for audio_video_recorder_server. please see [sample_audio_video_recorder_client.py](./node_scripts/sample_audio_video_recorder_client.py).

## Nodes

### audio_video_recorder

Node to record audio and video with given parameters.

#### Parameters

Node: `audio_video_recorder/audio_video_recorder`

##### Common parameters

- `queue_size` (`Int`, default: `100`)

  Queue size

- `file_name` (`String`, default: `/tmp/test.avi`)

  Output file location

- `file_format` (`String`, default: `avi`)

  Output file format (Only `avi` is supported now.)

##### Audio parameters

- `audio_format` (`String`, default: `mp3`)

  Audio format

- `audio_sample_format` (`String`, default: `S16LE`)

  Audio sample format

- `audio_channels` (`Int`, default: `1`)

  Number of audio channel

- `audio_depth` (`Int`, default: `16`)

  Audio depth

- `audio_sample_rate` (`Int`, default: `16000`)

  Audio sample rate

##### Video parameters

- `video_encoding` (`String`, default: `RGB`)

  Video encoding for `gstreamer`

- `video_height` (`Int`, default: `480`)

  Video image height

- `video_width` (`Int`, default: `640`)

  Video image width

- `video_framerate` (`Int`, default: `30`)

  Video frame rate

### audio_video_recorder_server

ROS Interface and recording task manager for audio_video_recorder.

#### Services

- `~start_record` (`audio_video_recorder/StartRecord`)

  Start a recording task

- `~stop_record` (`audio_video_recorder/StopRecord`)

  Stop a specified recording task

#### Publisher

- `~record_tasks` (`audio_video_recorder/RecordTaskArray`)

  Recording tasks currently running.
