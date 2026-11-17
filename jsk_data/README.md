# jsk_data

Data management tools used in JSK lab.
The package provides the `jsk_data` command line interface to upload and download data, a node to save synchronized topics
on request (`data_collection_server.py`), helpers for recording rosbags, and launch files to record and play back rosbags
of PR2, Fetch, Baxter, HRP2 and other robots.

## ROS 1 / ROS 2

The package builds for both ROS 1 (catkin) and ROS 2 (ament_cmake); `CMakeLists.txt` branches on `$ENV{ROS_VERSION}`.

| Item | ROS 1 | ROS 2 |
|---|---|---|
| `jsk_data` CLI | yes | yes |
| `data_collection_server.py` | yes | yes |
| `camera_coords_change_trigger.py` | yes | yes |
| Python module `jsk_data` | yes | yes |
| `rosbag_always.py`, `robot_description_saver.py`, `robot_description_loader.py` | yes | - |
| Shell scripts `copy-pr2-bagfiles`, `decompress`, `gen-gif-all`, `gopro-copy-files.sh`, `hrp2_rosbag_always.sh`, `drive-linux-x64` (x86_64 only) | yes | - |
| bash completion of `jsk_data` (env-hook) | yes | - |
| Launch files in `launch/` and `sample/` | yes | - |

There are no ROS 2 launch files; use `ros2 run jsk_data <exe>`.

## jsk_data command

`jsk_data` uploads, downloads and lists data on the lab's data server; it is designed for JSK lab members.
With `--public`, data is also put on / taken from the public Google Drive folder.

* `jsk_data ls [OPTIONS] [QUERY]` -- list data on the server.
* `jsk_data get [OPTIONS] FILENAME` -- download data.
* `jsk_data put [OPTIONS] FILENAME` -- upload data (`--public` also uploads to Google Drive, take care with private data).
* `jsk_data delete FILENAME` -- delete a public file (`--public` only).
* `jsk_data pubinfo [OPTIONS] FILENAME` -- show the info of public data.
* `jsk_data pubopen` -- open the public Google Drive folder in the browser.

See `jsk_data <sub command> --help` and [jsk_data_cli.md](../doc/jsk_data/scripts/jsk_data_cli.md).

## data_collection_server.py

Saves the latest messages of the given topics (and values of the given parameters) as
`{SAVE_DIR}/{TIMESTAMP}/{FILENAME}` when triggered.

Services:

* `~save_request` (`std_srvs/Trigger`) -- save once (`~method: request`).
* `~start_request` / `~end_request` (`std_srvs/Trigger`) -- start / stop periodic saving (`~method: timer`).

Main parameters: `~save_dir` (default `~/.ros`), `~topics`, `~params`, `~method` (`request`, `timer` or `all`),
`~message_filters`, `~approximate_sync`, `~queue_size`, `~slop`, `~timestamp_save_dir`.
Save types are `ColorImage`, `DepthImage`, `LabelImage` and `YAML` for topics, and `Text` and `YAML` for parameters.
See [data_collection_server.rst](../doc/jsk_data/node_scripts/data_collection_server.rst).

On ROS 1, `~topics` / `~params` are lists of dicts:

```xml
<node name="data_collection_server" pkg="jsk_data" type="data_collection_server.py">
  <rosparam>
    save_dir: ~/.ros/jsk_data
    topics:
      - name: /camera/rgb/image_raw
        msg_class: sensor_msgs/Image
        fname: image.png
        savetype: ColorImage
  </rosparam>
</node>
```

ROS 2 parameters cannot hold nested structures, so the same table is given as parallel string arrays
`topics_name`, `topics_msg_class`, `topics_fname`, `topics_savetype` (and `params_key`, `params_fname`, `params_savetype`):

```bash
ros2 run jsk_data data_collection_server.py --ros-args \
  -p save_dir:=~/.ros/jsk_data \
  -p topics_name:="['/camera/rgb/image_raw']" -p topics_msg_class:="['sensor_msgs/Image']" \
  -p topics_fname:="['image.png']" -p topics_savetype:="['ColorImage']"
ros2 service call /data_collection_server/save_request std_srvs/srv/Trigger
```

## camera_coords_change_trigger.py

Watches the pose of `head_mount_kinect_rgb_optical_frame` in `world` through TF and calls the `~trigger` service
(`std_srvs/Trigger`, e.g. remapped to `data_collection_server`'s `~save_request`) when the camera has moved by more than
`~delta_position` / `~delta_orientation` and has come to rest (`~velocify_position` / `~velocify_orientation`).

## rosbag_always.py (ROS 1)

Keeps recording rosbags even if roscore is restarted, and removes old bag files when the total size of `--save-dir` exceeds `--max-size` (in MB).

```bash
rosrun jsk_data rosbag_always.py --topics "/tf /joint_states" --size 1000 --save-dir /tmp/rosbag --max-size 10000
```

See [rosbag_always.md](../doc/jsk_data/scripts/rosbag_always.md) and `launch/rosbag_always_run_rosbag.launch`.

## Launch files (ROS 1)

* `launch/<robot>_record.launch` (`pr2`, `fetch`, `baxter`, `hrp2`, `urata`) -- record the robot's standard topics
  (TF, joint states, cameras, ...) into rosbags through `launch/common_record.launch`.
  Extra topics and `rosbag record` options are given with `other_topics` / `other_options`.
* `launch/<robot>_play.launch` (`pr2`, `fetch`, `baxter`, `hrp2`) -- play rosbags given by `bagfile_names` with the robot model
  and, optionally, the camera processing pipeline (`launch_openni`, ...) and RViz (`use_gui`).
* `launch/multisense_play.launch`, `launch/kinect2_bridge_play.launch`, `launch/point_cloud_reconstruction_from_multisense.launch`
  -- republish / reconstruct images and point clouds from recorded MultiSense or Kinect2 data.
* `launch/rosbag_always_run_rosbag.launch` -- run `rosbag_always.py` (`TOPICS`, `SIZE`, `OUTPUT`).
* `sample/sample_data_collection_server.launch` -- `data_collection_server.py` in the `request`, `timer` and `all` modes.

```bash
roslaunch jsk_data pr2_record.launch other_topics:="/foo /bar"
roslaunch jsk_data pr2_play.launch bagfile_names:=$HOME/data.bag
```

## Documentation

See [doc/jsk_data](../doc/jsk_data/index.rst).
