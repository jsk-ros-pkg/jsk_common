# jsk_rosbag_tools

Tools such as creating video from rosbag and compressing rosbag images.

## bag_to_video.py

Create video from rosbag.

### Usage

```
usage: bag_to_video.py [-h] [--out OUT] [--fps FPS] [--samplerate SAMPLERATE] [--channels CHANNELS] [--audio-topic AUDIO_TOPIC] [--image-topic IMAGE_TOPIC [IMAGE_TOPIC ...]] input_bagfile

rosbag to video

positional arguments:
  input_bagfile

optional arguments:
  -h, --help            show this help message and exit
  --out OUT, -o OUT     output directory path or filename.
                        If more than one --image-topic are specified,
                        this will be interpreted as a directory name.
                        Otherwise this is the file name.
  --fps FPS
  --samplerate SAMPLERATE, -r SAMPLERATE
                        sampling rate
  --channels CHANNELS   number of input channels
  --audio-topic AUDIO_TOPIC
  --image-topic IMAGE_TOPIC [IMAGE_TOPIC ...]
                        Topic name to extract.
```

### Example

```
rosrun jsk_rosbag_tools bag_to_video.py $(rospack find jsk_rosbag_tools)/samples/data/20220530173950_go_to_kitchen_rosbag.bag \
  --samplerate 16000 --channels 1 --audio-topic /audio \
  --image-topic /head_camera/rgb/throttled/image_rect_color/compressed \
  -o /tmp/20220530173950_go_to_kitchen_rosbag.mp4
```

## bag_to_audio.py

Create audio file from rosbag.

### Usage

```
usage: bag_to_audio.py [-h] [--out OUT] [--samplerate SAMPLERATE] [--channels CHANNELS] [--audio-topic AUDIO_TOPIC] input_bagfile

rosbag to audio

positional arguments:
  input_bagfile

optional arguments:
  -h, --help            show this help message and exit
  --out OUT, -o OUT     output filename. If `--audio-topic`_info is exists, you don't have to specify samplerate and channels.
  --samplerate SAMPLERATE, -r SAMPLERATE
                        sampling rate
  --channels CHANNELS   number of input channels
  --audio-topic AUDIO_TOPIC
```

### Example

```
rosrun jsk_rosbag_tools bag_to_audio.py $(rospack find jsk_rosbag_tools)/samples/data/20220530173950_go_to_kitchen_rosbag.bag \
  --samplerate 16000 --channels 1 --audio-topic /audio \
  -o /tmp/20220530173950_go_to_kitchen_rosbag.wav
```

## video_to_bag.py

Convert video file to bagfile.

### Usage

```
usage: video_to_bag.py [-h] [--out output_file] [--topic-name TOPIC_NAME] [--compress] [--no-progress-bar] inputvideo

Convert video to bag.

positional arguments:
  inputvideo

optional arguments:
  -h, --help            show this help message and exit
  --out output_file, -o output_file
                        name of the output bag file
  --topic-name TOPIC_NAME
                        Converted topic name.
  --compress            Compress Image flag.
  --no-progress-bar     Don't show progress bar.
```

### Example

```
rosrun jsk_rosbag_tools video_to_bag.py /tmp/output_bag/head_camera--slash--rgb--slash--throttled--slash--image_rect_color--slash--compressed-with-audio.mp4 \
    -o /tmp/output_bag/video.bag --compress
```

## compress_imgs.py

Convert `Image` messages to `CompressedImage` or `CompressedDepthImage`.

### Usage

```
usage: compress_imgs.py [-h] [--out OUT] [--compressed-topics [COMPRESSED_TOPICS [COMPRESSED_TOPICS ...]]] [--replace] [--no-progress-bar] input_bagfile

Convert Image messages to CompressedImage or CompressedDepthImage

positional arguments:
  input_bagfile         input bagfile path

optional arguments:
  -h, --help            show this help message and exit
  --out OUT, -o OUT     output bagfile path
  --compressed-topics [COMPRESSED_TOPICS [COMPRESSED_TOPICS ...]]
                        this image topics are compressed
  --replace
  --no-progress-bar     Don't show progress bar.
```

### Example

```
rosrun jsk_rosbag_tools compress_imgs.py $(rospack find jsk_rosbag_tools)/samples/data/20220530173950_go_to_kitchen_rosbag.bag \
  -o /tmp/20220530173950_go_to_kitchen_rosbag-compressed.bag
```

## tf_static_to_tf.py

Convert tf_static to tf and save it as a rosbag.

```
usage: tf_static_to_tf.py [-h] [--out OUT] [--no-progress-bar] input_bagfile

Convert tf_static to tf and save it as a rosbag

positional arguments:
  input_bagfile      input bagfile path

optional arguments:
  -h, --help         show this help message and exit
  --out OUT, -o OUT  output bagfile path
  --no-progress-bar  Don't show progress bar.
```

### Example

```
rosrun jsk_rosbag_tools tf_static_to_tf.py $(rospack find jsk_rosbag_tools)/samples/data/20220530173950_go_to_kitchen_rosbag.bag
```

### Note

`jsk_topic_tools` has a [static_tf_republisher.py](https://jsk-docs.readthedocs.io/projects/jsk_common/en/latest/jsk_topic_tools/scripts/static_tf_republisher.html) which republish `/tf_static` from a rosbag file.

`tf_static_to_tf.py` is an approach to rewrite the rosbag file.

## merge.py

Merges two bagfiles.

### Usage

```
usage: merge.py [-h] [--out output_file] [--topics TOPICS] [-i] main_bagfile bagfile

Merges two bagfiles.

positional arguments:
  main_bagfile          path to a bagfile, which will be the main bagfile
  bagfile               path to a bagfile which should be merged to the main bagfile

optional arguments:
  -h, --help            show this help message and exit
  --out output_file, -o output_file
                        name of the output file
  --topics TOPICS, -t TOPICS
                        topics which should be merged to the main bag
  -i                    reindex bagfile
```

### Example

```
rosrun jsk_rosbag_tools merge.py \
    $(rospack find jsk_rosbag_tools)/samples/data/20220530173950_go_to_kitchen_rosbag.bag \
    $(rospack find jsk_rosbag_tools)/samples/data/2022-05-07-hello-test.bag
```

## add_heatmap.py

Add a numeric MultiArray or single-channel Image topic as a heatmap `bgr8` Image topic in a new ROS 1/ROS 2 bag.
Each cell shows its numeric value by default.
No running ROS nodes are required.

### Usage

```
usage: add_heatmap.py [-h] [--out OUT] --input-topic INPUT_TOPIC
                      [--output-topic OUTPUT_TOPIC] [--rows ROWS]
                      [--cols COLS] [--frame-id FRAME_ID]
                      [--cell-size CELL_SIZE] [--hide-values]
                      [--precision PRECISION] [--colormap COLORMAP]
                      [--min-value MIN_VALUE] [--max-value MAX_VALUE]
                      input_bagfile

Add a numeric MultiArray or single-channel Image topic as a heatmap Image
(bgr8) topic in a new ROS 1/ROS 2 bag.

positional arguments:
  input_bagfile         Input ROS 1 bag file or ROS 2 rosbag2 SQLite3
                        directory

options:
  -h, --help            show this help message and exit
  --out OUT, -o OUT     Output path (must not exist). Default: <input-
                        stem>--heatmap.bag (ROS 1) or <input-
                        directory>--heatmap (ROS 2)
  --input-topic INPUT_TOPIC
                        Numeric MultiArray or single-channel Image topic to
                        visualize
  --output-topic OUTPUT_TOPIC
                        New heatmap Image topic (must not exist). Default:
                        INPUT_TOPIC/heatmap
  --rows ROWS           Number of rows; required with --cols for an empty
                        MultiArray layout
  --cols COLS           Number of columns; required with --rows for an empty
                        MultiArray layout
  --frame-id FRAME_ID   Frame ID for MultiArray input (default: empty). Image
                        headers are preserved
  --cell-size CELL_SIZE
                        Square cell size in pixels, at least 16 (default: 64)
  --hide-values         Hide cell values (shown by default)
  --precision PRECISION
                        Decimal places from 0 to 10 (default: 2). Long values
                        use scientific notation
  --colormap COLORMAP   OpenCV colormap name, e.g. jet, hot, viridis (default:
                        jet)
  --min-value MIN_VALUE
                        Lower color bound (default: each frame's finite
                        minimum)
  --max-value MAX_VALUE
                        Upper color bound (default: each frame's finite
                        maximum)

Use the ROS environment matching the input bag format. The input bag is not
modified. Set both color bounds to compare frames on a fixed scale; values are
clipped for coloring only.
```

### Example

ROS 1:

```bash
rosrun jsk_rosbag_tools add_heatmap.py /data/pressure.bag \
  --input-topic /sensor/pressure --output-topic /sensor/pressure/heatmap \
  --out /tmp/pressure--heatmap.bag
```

ROS 2:

```bash
ros2 run jsk_rosbag_tools add_heatmap.py /data/pressure_bag \
  --input-topic /sensor/pressure --output-topic /sensor/pressure/heatmap \
  --out /tmp/pressure_bag_heatmap
```

### Note

All original serialized messages and recording timestamps are retained, along with ROS 1 connection headers (including latching) or ROS 2 topic metadata (including offered QoS).
Added images use their source message's bag timestamp.
Image headers are preserved; MultiArrays use the bag timestamp and `--frame-id`.
Failed conversions do not leave a partial destination bag.

Matrix dimensions come from the two-dimensional MultiArray layout or Image dimensions.
Empty MultiArray layouts require `--rows` and `--cols`.
Row padding and Image byte order are respected.
NaN/Inf cells are gray.
Text shows original values, even when their color is clipped.
