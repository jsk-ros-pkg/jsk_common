#
# https://code.ros.org/trac/ros-pkg/ticket/4684
#
all: installed

SVN_DIR = move_base
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/navigation/tags/cturtle/move_base
SVN_PATCH = move_base.patch
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
