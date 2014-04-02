#
# https://code.ros.org/trac/wg-ros-pkg/ticket/5118
#
all: installed

GIT_DIR = pr2_apps
GIT_URL = https://github.com/PR2/pr2_apps.git
GIT_PATCH = pr2_teleop_yaw.patch
GIT_REVISION = e2ad822918e7b6e2913446a661518fe15e54cbab
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	[ `rospack find pr2_msgs` ] && (cd $(GIT_DIR)/pr2_teleop_general && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake ) || echo "sudo apt-get install ros-$ROS_DISTRO-pr2-msgs"
	touch installed

clean:
	-cd $(GIT_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(GIT_DIR)
