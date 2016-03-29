jsk\_data
=========

A stack for data management tools which are used in JSK lab.


`$ jsk_data` command
--------------------

`jsk_data` is command line interface to rename, upload and download data, and
this is fully designed for JSK lab members.
(this command communicates with internal server in lab)

There are following sub commands. See `jsk_data [sub command] --help` for more detail.

* `ls`: List data on the server.

    Usage is `jsk_data ls [OPTIONS] [QUERY]`.

* `get`: Download data from the server.

    Usage is `jsk_data put [OPTIONS] FILENAME`.

* `put`: Upload data to the server.

    Usage is `jsk_data put [OPTIONS] FILENAME`.
    With `--public` option, it also uploads to
    [public Google Drive folder](https://drive.google.com/folderview?id=0B9P1L--7Wd2vUGplQkVLTFBWcFE),
    so please care about it when you handle secure data.

* `delete`: Delete file on the server.

    Usage is `jsk_data delete FILENAME`.
    It only supports with `--public` option.
    If you want to delete private data, delete it manually by logging in aries via ssh.

* `pubinfo`: Show public data info.

    Usage is `jsk_data pubinfo [OPTIONS] FILENAME`.
    Downloading large file with `wget` or `curl` from Google Drive can be failed.
    (see [here](http://stackoverflow.com/questions/25010369/wget-curl-large-file-from-google-drive))
    Please run `sudo pip install gdown` and use it at that time. (`Usage: gdown [URL] -O [FILENAME]`)

* `pubopen`: Open Google Drive Folder where public data is uploaded

    Usage is `jsk_data pubopen`.


**Screencast**

![](./images/jsk_data_cli_screencast.gif)

rosbag_always.py
----------------
rosbag_always.py can record bag files even if roscore is restarted.
It also removes old bag files if recorded bag files exceed specified size.

```
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


rosbag\_for\_rviz.py
------------------

This is to record all topics subscribed from rviz node, in order to record topics for playing later
and creating demo movies.

**Note**
Node name of rviz should start with '/rviz'.

Usage:

```bash
$ rviz -d $(rospack find rviz)/default.rviz
$ rosrun jsk_data rosbag_for_rviz.py
Found rviz node: /rviz_1459260977125132860
[ INFO] [1459260987.192261200]: Subscribing to /tf
[ INFO] [1459260987.194573348]: Subscribing to /tf_static
[ INFO] [1459260987.196886950]: Recording to 2016-03-29-23-16-27.bag.
```
