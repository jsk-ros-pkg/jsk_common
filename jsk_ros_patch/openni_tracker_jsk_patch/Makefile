#
# https://code.ros.org/trac/ros-pkg/ticket/xxxx
#
all: installed

HG_DIR = build_openni_tracker
HG_URL = http://kforge.ros.org/openni/openni_tracker
#HG_PATCH = openni_tracker_use_calibfile.patch
HG_PATCH = openni_tracker_auto_calibration.patch
HG_REVISION = openni_tracker-0.1.0
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR) patched
	cd $(HG_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	-cd $(HG_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(HG_DIR) rospack_nosubdirs

