#!/bin/sh

# Script to download maxwell scripts under ~/maxwell

TARGET_DIR=$HOME/maxwell
SCRIPTS="http://iwl.com/drc2015/drcctrl.py http://iwl.com/drc2015/dce_ctl.py http://iwl.com/drc2015/servercomm.py http://iwl.com/drc2015/track.py http://iwl.com/drc2015/starttrack.sh http://iwl.com/drc2015/blackball-16.png http://iwl.com/drc2015/blueball-16.png http://iwl.com/drc2015/greenball-16.png http://iwl.com/drc2015/redball-16.png http://iwl.com/drc2015/yellowball-16.png http://iwl.com/drc2015/startstdiserver.sh http://iwl.com/drc2015/2015-2-2_Degraded_Comms_Schedule_Example.csv http://iwl.com/drc2015/schedule.yaml"

mkdir -p $TARGET_DIR
for s in $SCRIPTS
do
  echo downloading $s
  (cd $TARGET_DIR && wget -q --timestamping $s)
done

(cd $TARGET_DIR && chmod +x starttrack.sh)

echo "========================================================"
echo You may need to edit starttrack.sh to specify
echo "  BKOUTFILE=2015-2-2_Degraded_Comms_Schedule_Example.csv"
