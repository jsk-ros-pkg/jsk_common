#
# https://code.ros.org/trac/ros-pkg/ticket/xxxx
#
include $(shell rospack find mk)/cmake.mk

all: installed

SVN_DIR = build/image_view
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/image_pipeline/trunk/image_view
SVN_PATCH = image_saver.patch
SVN_REVISION= -r 39154
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p `dirname $(SVN_DIR)`
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH rosmake --rosdep-install
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
