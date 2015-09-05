##
## make large KEYWORD=2014_05* ;; to download only interested data
##
all: small

LOGIN_USER=$(shell if [ "${SSH_USER}" != "" ]; then echo ${SSH_USER}; else echo ${USER}; fi)
DATA_DIR=$(shell if [ -e /home/jsk/ros/data ] ; then echo "/home/jsk/ros/data"; else echo "${LOGIN_USER}@aries.jsk.t.u-tokyo.ac.jp:/home/jsk/ros/data"; fi)

.PHONY : small large

large-list:
	@echo "***\n*** Consider using KEYWORD option to get the list of large data\n***"
	@echo "*** for example... "
	@echo "*** > make large-list KEYWORD=2014_05*"
	@ssh ${LOGIN_USER}@aries.jsk.t.u-tokyo.ac.jp ls -sh /home/jsk/ros/data/large/${KEYWORD}
small-list:
	@ssh ${LOGIN_USER}@aries.jsk.t.u-tokyo.ac.jp ls -sh /home/jsk/ros/data/small/${KEYWORD}

large:
	@echo "***\n*** Consider using KEYWORD option to download large data\n***"
	@echo "*** for example... "
	@echo "*** > make large KEYWORD=2014_05*"
	@echo "*** current large data list is ..."
	@ssh ${LOGIN_USER}@aries.jsk.t.u-tokyo.ac.jp ls /home/jsk/ros/data/large
	@echo "---"
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/large/${KEYWORD} large/ || exit 0
	bash `rospack find jsk_data`/scripts/decompress large || exit 0
	sh `rospack find jsk_data`/scripts/gen-gif-all large || exit 0

small:
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/small/${KEYWORD} small/ || exit 0
	bash `rospack find jsk_data`/scripts/decompress small || exit 0
	sh `rospack find jsk_data`/scripts/gen-gif-all small || exit 0

pkls:
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/pkls . || exit 0

pcds:
	rsync -avz --progress -e "ssh -o StrictHostKeyChecking=no" --bwlimit=100000 ${DATA_DIR}/pcds . || exit 0
