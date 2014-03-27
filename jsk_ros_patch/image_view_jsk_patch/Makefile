#
# https://code.ros.org/trac/ros-pkg/ticket/5529
#
EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1
include $(shell rospack find mk)/cmake.mk

all: installed

GIT_DIR = build/image_pipeline
GIT_URL = https://github.com/ros-perception/image_pipeline/
GIT_REVISION = image_pipeline-1.8.4
GIT_PATCH = image_saver-filename_format_param.patch image_saver-servicecall.patch image_saver-encoding.patch image_saver-update-param.patch image_saver-file_saving.patch
include $(shell rospack find mk)/git_checkout.mk

installed: $(SVN_DIR) patched
	mkdir -p `dirname $(GIT_DIR)`
	cd $(GIT_DIR)/image_view && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make
	cp $(GIT_DIR)/image_view/bin/image_saver `pwd`
	touch installed

clean:
	cd $(GIT_DIR) && ROS_PACKAGE_PATH=`pwd`:$$ROS_PACKAGE_PATH make clean
	rm -rf installed patched

wipe: clean
	rm -rf $(GIT_DIR)
