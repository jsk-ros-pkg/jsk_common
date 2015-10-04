jsk\_data
=========

A stack for data management tools which are used in JSK lab.


`$ jsk_data` command
--------------------

`jsk_data` is command line interface to rename, upload and download data, and
this is fully designed for JSK lab members.
(this command communicates with internal server in lab)

There are following sub commands. See `jsk_data [sub command] --help` for more detail.

* `ls`: List data in the server.

    Usage is `jsk_data ls [OPTIONS] [QUERY]`.

* `get`: Download data from the server.

    Usage is `jsk_data put [OPTIONS] FILENAME`.

* `put`: Upload data to the server.

    Usage is `jsk_data put [OPTIONS] FILENAME`.  
    With `--public` option, it also uploads to
    [public Google Drive folder](https://drive.google.com/folderview?id=0B9P1L--7Wd2vUGplQkVLTFBWcFE),
    so please care about it when you handle secure data.  

* `pubinfo`: Show public data info.

    Usage is `jsk_data pubinfo [OPTIONS] FILENAME`.  

**Screencast**

![](./images/jsk_data_cli_screencast.gif)
