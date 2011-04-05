#
# https://code.ros.org/trac/ros-pkg/ticket/xxxx
#
all: installed

SVN_DIR = pr2_teleop_general
SVN_URL = https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_apps/trunk/pr2_teleop_general
SVN_PATCH = 
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cp -f `rospack find move_base`/../rosdep.yaml $(SVN_DIR)
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake --rosdep-install
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
