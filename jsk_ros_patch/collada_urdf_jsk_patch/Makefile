#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

HG_DIR = robot_model-1.5.1_hg
HG_URL = https://kforge.ros.org/robotmodel/robot_model
HG_REVISION = robot_model-1.5.1
#HG_PATCH = color.patch
#HG_PATCH = cyliner_box_sphere.patch rosdep.patch
HG_PATCH = rosdep.patch
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR)
	(cd $(HG_DIR) && hg revert --all)
	$(foreach PATCH, $(HG_PATCH), patch -d $(HG_DIR) -p1 < $(PATCH) &&) echo patched
	ROS_PACKAGE_PATH=`pwd`/$(HG_DIR):$$ROS_PACKAGE_PATH rosmake collada_urdf
	touch installed

clean:
	-cd $(HG_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched
	if [ -e $(HG_DIR)/rosdep.yaml ]; then patch -R -d $(HG_DIR) -p0 < rosdep.patch; fi

wipe: clean
	rm -rf $(HG_DIR)