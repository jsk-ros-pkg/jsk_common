#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

HG_DIR = build/robot_model
HG_URL = https://kforge.ros.org/robotmodel/robot_model
HG_REVISION = robot_model-`rosversion robot_model`
HG_PATCH = cyliner_box_sphere.patch set_url_name.patch
include $(shell rospack find mk)/hg_checkout.mk

# https://code.ros.org/trac/ros/ticket/3748
$(HG_DIR):
	mkdir -p `dirname $(HG_DIR)`; hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg update $(HG_BRANCH) && hg update $(HG_REVISION)
	touch rospack_nosubdirs

installed:$(HG_DIR) patched
	ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/collada_urdf
	cp `pwd`/$(HG_DIR)/collada_urdf/bin/urdf_to_collada .
	touch installed

clean:
	if [ -f $(HG_DIR)/collada_urdf ] ; then ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/collada_urdf clean; fi
	rm -rf installed patched
	rm -f urdf_to_collada

wipe: clean
	rm -rf $(HG_DIR)
