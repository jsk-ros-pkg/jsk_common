#
# https://code.ros.org/trac/ros-pkg/ticket/xxxx
#
all: installed

HG_DIR = build_arm_navigation
HG_URL = http://kforge.ros.org/armnavigation/armnavigation
HG_PATCH = move_arm.patch
HG_REVISION = fuerte
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR) patched
	cd $(HG_DIR)/move_arm && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch rospack_nosubdirs
	cp `pwd`/$(HG_DIR)/move_arm/bin/planning_components_visualizer planning_components_executor
	touch installed

clean:
	-cd $(HG_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched planning_components_executor

wipe: clean
	rm -rf $(HG_DIR) rospack_nosubdirs
