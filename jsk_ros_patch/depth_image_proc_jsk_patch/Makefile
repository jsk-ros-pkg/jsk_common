include $(shell rospack find mk)/cmake.mk

all: installed

HG_DIR = build/openni_kinect
HG_URL = http://kforge.ros.org/openni/openni_ros
HG_REVISION = openni_kinect-`rosversion openni_kinect`
HG_PATCH = openni_zoom.patch
include $(shell rospack find mk)/hg_checkout.mk

# https://code.ros.org/trac/ros/ticket/3748
$(HG_DIR):
	mkdir -p `dirname $(HG_DIR)`; hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg update $(HG_BRANCH) && hg update $(HG_REVISION)
	touch rospack_nosubdirs

installed:$(HG_DIR) patched
	ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/depth_image_proc:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/depth_image_proc
	cp -r `pwd`/$(HG_DIR)/openni_launch/launch .
	touch installed

clean:
	if [ -f $(HG_DIR)/openni_kinect ] ; then ROS_PACKAGE_PATH=`pwd`/$(HG_DIR)/depth_image_proc:$$ROS_PACKAGE_PATH make -C `pwd`/$(HG_DIR)/depth_image_proc clean; fi
	rm -rf installed patched
	rm -rf launch

wipe: clean
	rm -rf $(HG_DIR)
