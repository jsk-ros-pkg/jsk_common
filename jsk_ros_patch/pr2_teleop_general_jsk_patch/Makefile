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
	[ `rospack find pr2_msgs` ] && (cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake ) || echo "sudo apt-get install ros-$ROS_DISTRO-pr2-msgs"
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
