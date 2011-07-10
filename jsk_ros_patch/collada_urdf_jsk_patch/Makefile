#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

HG_DIR = collada_urdf_hg
HG_URL = https://kforge.ros.org/robotmodel/robot_model
HG_REVISION = robot_model-1.5.1
#HG_PATCH = color.patch
#HG_PATCH = cyliner_box_sphere.patch rosdep.patch
HG_PATCH = rosdep.patch
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR) patched
	cd $(HG_DIR)/collada_urdf && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	-cd $(HG_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(HG_DIR)