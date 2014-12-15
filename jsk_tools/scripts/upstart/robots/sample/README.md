## example

1. ``roscd jsk_tools/scripts/upstart/robots/sample``
1. ``find *.conf | xargs sed -i "s/eisoku/$USER/g" *.conf``
1. ``rosrun jsk_tools setup_upstart_jobs.bash sample install``
1. ``sudo reboot``

## check
1. ``byobu``

Open a new terminal
1. ``sudo initctl stop test1``
1. ``sudo initctl start test1``
