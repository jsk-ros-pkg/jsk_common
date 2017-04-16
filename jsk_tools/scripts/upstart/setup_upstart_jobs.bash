#!/bin/bash

# Condition check
if [ $# -ne 2 ]; then
    echo "USAGE : rosrun jsk_tools setup_upstart_jobs.bash staro install (and reboot)"
    echo "USAGE : rosrun jsk_tools setup_upstart_jobs.bash hrp2 uninstall (and reboot)"
    exit 1
elif [ ! -e `rospack find jsk_tools`/scripts/upstart/robots/$1 ]; then
    echo "No such directory : "$1
    exit 1
fi


# install or uninstall
if [ $2 = "install" ]; then
    for j in `rospack find jsk_tools`/scripts/upstart/robots/$1/*.conf
    do
        sudo cp -f $j /etc/init/ # upstart does not support symbolic links
        echo "sudo cp -f "$j" /etc/init"
    done
elif [ $2 = "uninstall" ]; then
    for j in `rospack find jsk_tools`/scripts/upstart/robots/$1/*.conf
    do
        sudo rm -f /etc/init/${j##*/}
        echo "sudo rm -f /etc/init/"${j##*/}
    done
fi
