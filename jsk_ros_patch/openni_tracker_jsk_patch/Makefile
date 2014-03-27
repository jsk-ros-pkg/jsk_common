#
# https://code.ros.org/trac/ros-pkg/ticket/xxxx
#
all: installed

GIT_DIR = build_openni_tracker
GIT_URL = http://github.com/ros-drivers/openni_tracker
#GIT_PATCH = openni_tracker_use_calibfile.patch
GIT_PATCH = openni_tracker_auto_calibration.patch
GIT_REVISION = openni_tracker-0.1.0
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	cd $(GIT_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	-cd $(GIT_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(GIT_DIR) rospack_nosubdirs

