roscore_regardless.py
---------------------

This script always checks roscore liveness and automatically run and kill a program.

```
rosrun jsk_tools roscore_regardless.py rostopic echo /foo
```

## Usage
```
$ rosrun jsk_tools roscore_regardless.py -h
usage: roscore_regardless.py [-h] [--respawn] [--timeout TIMEOUT] ...

positional arguments:
  commands

  optional arguments:
    -h, --help         show this help message and exit
    --respawn, -r      respawn if child process stops
    --timeout TIMEOUT  Timeout to verify if rosmaster is alive by ping command
                       in seconds.
```
