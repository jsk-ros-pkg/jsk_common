#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

HG_DIR = build/robot_model
HG_URL = http://kforge.ros.org/robotmodel/robot_model
HOGE=$(shell rosversion -d)
ifeq ($(shell rosversion -d),groovy)
HG_REVISION = `rosversion collada_urdf`
HG_PATCH = collada.patch cyliner_box_sphere.patch set_url_name.patch cmake_catkin.patch
HG_BIN_DIR  = `pwd`/$(HG_DIR)/collada_urdf/build/devel/lib/collada_urdf
HG_ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH
else
HG_REVISION = robot_model-`rosversion robot_model`
HG_PATCH = cyliner_box_sphere.patch set_url_name.patch cmake_rosbuild.patch collada-dom.patch
HG_BIN_DIR  = `pwd`/$(HG_DIR)/collada_urdf/bin
HG_ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:`pwd`/$(HG_DIR)/colladadom:$$ROS_PACKAGE_PATH
endif
include $(shell rospack find mk)/hg_checkout.mk

# https://code.ros.org/trac/ros/ticket/3748
$(HG_DIR):
	mkdir -p `dirname $(HG_DIR)`; hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg update $(HG_BRANCH) && hg update $(HG_REVISION)
	touch rospack_nosubdirs

installed:$(HG_DIR) patched
	ROS_PACKAGE_PATH=$(HG_ROS_PACKAGE_PATH) rosmake collada_urdf
	cp $(HG_BIN_DIR)/urdf_to_collada .
	touch installed

clean:
	if [ -f $(HG_DIR)/collada_urdf ] ; then ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/collada_urdf clean; fi
	rm -rf installed patched
	rm -f urdf_to_collada

wipe: clean
	rm -rf $(HG_DIR)
