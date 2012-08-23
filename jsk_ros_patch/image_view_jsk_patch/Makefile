#
# https://code.ros.org/trac/ros-pkg/ticket/5529
#
include $(shell rospack find mk)/cmake.mk

all: installed

SVN_DIR = build/image_view
SVN_URL = https://code.ros.org/svn/ros-pkg/stacks/image_pipeline/tags/electric/image_proc/
SVN_PATCH = image_saver-filename_format_param.patch image_saver-servicecall.patch image_saver-encoding.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p `dirname $(SVN_DIR)`
	cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	touch installed

clean:
	-cd $(SVN_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(SVN_DIR)
