#
# https://kforge.ros.org/openni/trac/ticket/74
#
EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1
include $(shell rospack find mk)/cmake.mk

all: installed

GIT_DIR = build/image_pipeline
GIT_URL = https://github.com/ros-perception/image_pipeline/
GIT_REVISION = image_pipeline-1.8.4
GIT_PATCH = openni_zoom.patch manifest.xml.patch # to avoid common_rosdeps
include $(shell rospack find mk)/git_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p `dirname $(GIT_DIR)`
	cd $(GIT_DIR)/depth_image_proc && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	cd $(GIT_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(GIT_DIR)
