#!/bin/bash

if [ "$ROSDEP_UPDATE_QUIET" = "true" ]; then
    ROSDEP_ARGS=>/dev/null
fi

trap 'find -L . -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i mv `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml' 1 2 3 15

find -L . -name package.xml -exec dirname {} \; | xargs -n 1 -i find {} -name manifest.xml | xargs -n 1 -i mv {} {}.deprecated # rename manifest.xml for rosdep install
rosdep install -r -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y $ROSDEP_ARGS
find -L . -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i mv `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml
