#
all: urdf_to_collada

GIT_DIR = build/robot_model/src
GIT_URL = git://github.com/ros/robot_model.git
GIT_REVISION = ${SOURCE_DISTRO}-devel
PATCH_DIR = $(CURDIR)
GIT_PATCH = ${PATCH_DIR}/use_assimp_devel.patch ${PATCH_DIR}/collada_urdf_latest_gazebo.patch
BUILD_BIN_DIR  = build/robot_model/devel/lib/collada_urdf
include $(shell rospack find mk)/git_checkout.mk

disable_ssl:
	git config --global http.sslVerify false

urdf_to_collada: disable_ssl $(GIT_DIR) patched
	(cd build/robot_model; PKG_CONFIG_PATH=`rospack find assimp_devel`/lib/pkgconfig:${PKG_CONFIG_PATH} catkin_make --pkg collada_urdf --force-cmake)
	cp $(BUILD_BIN_DIR)/urdf_to_collada .
	cp $(BUILD_BIN_DIR)/collada_to_urdf .

clean:
	if [ -f build/robot_model/build ] ; then catkin_make clean; fi
	rm -fr build/robot_model/build
	rm -rf installed patched
	rm -f urdf_to_collada

wipe: clean
	rm -rf $(SRC_DIR)
