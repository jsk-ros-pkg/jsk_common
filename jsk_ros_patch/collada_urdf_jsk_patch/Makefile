#
# https://code.ros.org/trac/ros-pkg/ticket/4276
#
all: urdf_to_collada


ifneq ($(shell rosversion -d),fuerte)
#HG_REVISION = `rosversion collada_urdf`
## robot_model moved to github, 1.9.32 was last version in kforge
GIT_DIR = build/robot_model
GIT_URL = https://github.com/ros/robot_model.git
GIT_REVISION = groovy-devel
GIT_PATCH = set_url_name_groovy.patch cmake_catkin.patch
BUILD_BIN_DIR  = `pwd`/$(GIT_DIR)/collada_urdf/build/devel/lib/collada_urdf
include $(shell rospack find mk)/git_checkout.mk
SRC_DIR = $(GIT_DIR)
else ## fuerte
GIT_DIR = build/robot_model
GIT_URL = https://github.com/ros/robot_model.git
GIT_REVISION = robot_model-`rosversion robot_model`
GIT_PATCH = cyliner_box_sphere.patch set_url_name.patch cmake_rosbuild.patch collada-dom.patch downloads.patch
BUILD_BIN_DIR  = `pwd`/$(GIT_DIR)/collada_urdf/bin
include $(shell rospack find mk)/git_checkout.mk
SRC_DIR = $(GIT_DIR)
endif

urdf_to_collada:$(SRC_DIR) patched
ifneq ($(shell rosversion -d),fuerte)
	ROS_PACKAGE_PATH=`pwd`/$(SRC_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(SRC_DIR)/collada_urdf
	HG_ROS_PACKAGE_PATH=
else
	ROS_PACKAGE_PATH=`pwd`/$(SRC_DIR)/collada_urdf:`pwd`/$(SRC_DIR)/colladadom:$$ROS_PACKAGE_PATH make -C `pwd`/$(SRC_DIR)/colladadom
	ROS_PACKAGE_PATH=`pwd`/$(SRC_DIR)/collada_urdf:`pwd`/$(SRC_DIR)/colladadom:$$ROS_PACKAGE_PATH make -C `pwd`/$(SRC_DIR)/collada_urdf
endif
	cp $(BUILD_BIN_DIR)/urdf_to_collada .

clean:
	if [ -f $(SRC_DIR)/collada_urdf ] ; then ROS_PACKAGE_PATH=`pwd`/$(SRC_DIR)/collada_urdf:$$ROS_PACKAGE_PATH make -C `pwd`/$(SRC_DIR)/collada_urdf clean; fi
	rm -rf installed patched
	rm -f urdf_to_collada

wipe: clean
	rm -rf $(SRC_DIR)
