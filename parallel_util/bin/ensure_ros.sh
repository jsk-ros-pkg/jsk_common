#!/bin/sh

if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    echo false
    exit 1
fi

dpkg-query -W  ros-diamondback-desktop-full > /dev/null
if [ $? ]; then
    echo true
    exit 0
else
    echo false
    exit 1
fi

