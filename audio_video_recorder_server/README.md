# Audio Video Recorder Server

This package provides ROS Topic/Service Interface to [audio_video_recorder](../audio_video_recorder) and python client library.

## How to use demo

`demo.launch` shows how to use this package.
Before launching, please make sure your PC has a usb camera and a microphone.

```
roslaunch audio_video_recorder_server demo.launch
```

This launch will launch `audio_video_recorder_server` node and an example client node with `AudioVideoRecoderClient` instance. The client script will start 2 recording tasks simultaneously, and after finishing, there will be 2 output video files. (`/tmp/audio_video_recorder_server_demo_02.avi` and `/tmp/audio_video_recorder_server_demo_02.avi`)

See [demo.launch](./launch/demo.launch) and [client_demo.py](./node_scripts/client_demo.py) for details.

## Node

### audio_video_recorder_server

#### Services

- `~start_record` (type: `audio_video_recorder_server/StartRecord`)

Start a recording task.

- `~stop_record` (type: `audio_video_recorder_server/StopRecord`)

Stop an specified recoding task.

#### Topics

- `~record_tasks` (type: `audio_video_recorder_server/RecordTaskArray)

  Recoding tasks currently running.
