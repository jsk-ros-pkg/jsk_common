#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: installed

SVN_DIR = collada_urdf_svn
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/robot_model/tags/cturtle/collada_urdf
SVN_PATCH = color.patch cyliner_box_sphere.patch rosdep.patch
include $(shell rospack find mk)/svn_checkout.mk

patched2: $(SVN_PATCH) Makefile
ifneq ($(strip $(SVN_PATCH)),)
	-rm -f $(SVN_DIR)/rosdep.yaml
	svn revert -R $(SVN_DIR)
endif
	-cd $(SVN_DIR) && $(SVN_CMDLINE) up $(SVN_REVISION)
	$(foreach PATCH, $(SVN_PATCH), patch -d $(SVN_DIR) -p0 < $(PATCH) && ) echo patched
	touch rospack_nosubdirs
	touch patched

installed: $(SVN_DIR) patched2
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)