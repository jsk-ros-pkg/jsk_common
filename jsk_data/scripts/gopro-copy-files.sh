#!/bin/bash

DIRS=/media/*

HAVE_GO_PRO=False
for d in $DIRS
do
    if [ -e $d/DCIM -a -e $d/MISC -a -e $d/Android -a -e $d/customized-capability.xml -a -e $d/default-capability.xml -a -e $d/DCIM/100GOPRO ]; then
        echo $d seems to be a gopro directory
        HAVE_GO_PRO=True
    fi
done

if [ $HAVE_GO_PRO == True ]; then
    echo -n "Please Input a user to ssh: "
    read ssh_user
    echo -n "Do you want to remove the files?[Yes/No]: "
    read reamove_files
    if [ "$remove_files" = "Yes" -o "$remove_files" = "yes" -o "$remove_files" = "y" ]; then
        RSYNC_OPTION="--remove-source-files"
    fi
    for d in $DIRS
    do
        if [ -e $d/DCIM -a -e $d/MISC -a -e $d/Android -a -e $d/customized-capability.xml -a -e $d/default-capability.xml -a -e $d/DCIM/100GOPRO ]; then
            rsync --progress $RSYNC_OPTION -avz $d/DCIM/100GOPRO/ $ssh_user@aries.jsk.t.u-tokyo.ac.jp:/export/andromeda/GOPRO/$(date '+%F-%H')/
        fi
    done
fi
