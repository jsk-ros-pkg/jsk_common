roscore_regardless.py
---------------------

This script always checks roscore liveness and automatically run and kill a program.

```
rosrun jsk_tools roscore_regardless.py rostopic echo /foo
```

## Usage
```
$ rosrun jsk_tools roscore_regardless.py -h
usage: roscore_regardless.py [-h] [--respawn] [--timeout TIMEOUT]
                             [--ping-trials PING_TRIALS]
                             [--sigint-timeout SIGINT_TIMEOUT]
                             [--sigterm-timeout SIGTERM_TIMEOUT]
                             ...

positional arguments:
  commands

optional arguments:
  -h, --help            show this help message and exit
  --respawn, -r         respawn if child process stops
  --timeout TIMEOUT     Timeout to verify if rosmaster is alive by ping
                        command in seconds
  --ping-trials PING_TRIALS
                        If ping fails PING-TRIALS times, master is regarded as
                        dead
  --sigint-timeout SIGINT_TIMEOUT
                        Timeout to escalete from sigint to sigterm to kill
                        child processes
  --sigterm-timeout SIGTERM_TIMEOUT
                        Timeout to escalete from sigterm to sigkill to kill
                        child processes
```
