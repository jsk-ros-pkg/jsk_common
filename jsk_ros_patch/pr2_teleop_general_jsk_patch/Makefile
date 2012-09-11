#
# https://code.ros.org/trac/wg-ros-pkg/ticket/5118
#
all: installed

SVN_DIR = pr2_teleop_general
SVN_URL = https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_apps/trunk/pr2_teleop_general
SVN_PATCH = pr2_teleop_general.patch
SVN_REVISION= -r 53700
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	wget https://kforge.ros.org/navigation/navigation/file/0e595d0774c1/rosdep.yaml -O $(SVN_DIR)/rosdep.yaml
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
