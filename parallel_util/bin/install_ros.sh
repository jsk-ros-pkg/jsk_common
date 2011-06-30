#!/bin/sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu lucid main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get install ros-diamondback-desktop-full
