#
# https://code.ros.org/trac/ros-pkg/ticket/4107
#
all: installed

SVN_DIR = sound_play
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/sound_drivers/tags/cturtle/sound_play
SVN_PATCH = sound_play.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cp -f `rospack find sound_play`/../rosdep.yaml $(SVN_DIR)
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake --rosdep-install
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)