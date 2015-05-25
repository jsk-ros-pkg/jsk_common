#!/bin/bash

if [ "`cat /etc/sysctl.conf | grep net.core.rmem_max`" = "" ]; then
    echo 'net.core.rmem_max = 4259840' >> /etc/sysctl.conf
    sysctl -p
    echo "changed kernel parameter"
else
    echo "nothing to change"
fi
