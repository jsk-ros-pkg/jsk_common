#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

HG_DIR = build/robot_model-1.6.3
HG_URL = https://kforge.ros.org/robotmodel/robot_model
HG_REVISION = robot_model-1.6.3
HG_PATCH = cyliner_box_sphere.patch
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR)
	ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/collada_urdf
	cp `pwd`/$(HG_DIR)/collada_urdf/bin/urdf_to_collada .
	touch installed

clean:
	ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/collada_urdf clean
	rm -rf installed patched
	rm urdf_to_collada

wipe: clean
	rm -rf $(HG_DIR)
