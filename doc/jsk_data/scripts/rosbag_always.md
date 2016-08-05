# rosbag\_always.py

rosbag\_always.py can record bag files even if roscore is restarted.
It also removes old bag files if recorded bag files exceed specified size.

```bash
$ rosrun jsk_data rosbag_always.py -h
usage: rosbag_always.py [-h] --topics TOPICS --size SIZE --save-dir SAVE_DIR
                        --max-size MAX_SIZE

rosbag record regardless of rosmaster status

optional arguments:
  -h, --help           show this help message and exit
  --topics TOPICS      topics to record
  --size SIZE          size of each rosbag
  --save-dir SAVE_DIR  directory to store rosbag
  --max-size MAX_SIZE  maximum size of rosbags in save_dir
```
