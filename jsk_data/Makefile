all: small

DATA_DIR=$(shell if [ -e /home/jsk/ros/data ] ; then echo "/home/jsk/ros/data"; else echo "$$USER@aries.jsk.t.u-tokyo.ac.jp:/home/jsk/ros/data"; fi)

.PHONY : small large

large:
	rsync -avz --progress --bwlimit=100000 ${DATA_DIR}/large .
	sh `rospack find jsk_data`/scripts/gen-gif-all large

small:
	rsync -avz --progress --bwlimit=100000 ${DATA_DIR}/small .
	sh `rospack find jsk_data`/scripts/gen-gif-all small
