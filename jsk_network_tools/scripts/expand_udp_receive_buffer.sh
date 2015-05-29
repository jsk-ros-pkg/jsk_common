#!/bin/bash

set -eu

if [ "`cat /etc/sysctl.conf | grep net.core.rmem_max`" = "" ]; then
    echo 'net.core.rmem_max = 782237696' >> /etc/sysctl.conf
    sysctl -p
else
    sed -i -e "s/net.core.rmem_max = \(.*\)/net.core.rmem_max = 782237696/g" /etc/sysctl.conf
    sysctl -p
fi
