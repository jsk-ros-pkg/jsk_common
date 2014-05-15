##
## make large KEYWORD=2014_05* ;; to download only interested data
##
all: small

DATA_DIR=$(shell if [ -e /home/jsk/ros/data ] ; then echo "/home/jsk/ros/data"; else echo "$$USER@aries.jsk.t.u-tokyo.ac.jp:/home/jsk/ros/data"; fi)

.PHONY : small large

large:
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/large/${KEYWORD} large/ || exit 0
	sh `rospack find jsk_data`/scripts/gen-gif-all large || exit 0

small:
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/small/${KEYWORD} small/ || exit 0
	sh `rospack find jsk_data`/scripts/gen-gif-all small || exit 0
