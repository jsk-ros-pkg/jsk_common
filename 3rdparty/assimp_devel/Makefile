all: assimp_devel

INSTALL_DIR=`rospack find assimp_devel`
TARBALL = v3.1.1.zip
TARBALL_URL = https://github.com/assimp/assimp/archive/v3.1.1.zip
UNPACK_CMD = unzip -q
PATCH_DIR = $(CURDIR)
TARBALL_PATCH = ${PATCH_DIR}/assimp_git.technique.patch ${PATCH_DIR}/assimp_git.obj_export.patch ${PATCH_DIR}/assimp_devel.patch
MK_DIR    = $(shell rospack find mk)

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)
# CPU_NUM=$(shell grep -c processor /proc/cpuinfo)
# PARALLEL_JOB=$(shell if `expr $(CPU_NUM) \> 4 > /dev/null`;then echo 4; else echo ${CPU_NUM}; fi)

SOURCE_DIR=build/assimp-3.1.1

include $(MK_DIR)/download_unpack_build.mk

.PHONY: assimp_devel

build: assimp_devel

assimp_devel: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && mkdir -p $(BUILDDIR) && cd $(BUILDDIR) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) .. && make install
	if [ -e $(INSTALL_DIR)/lib/pkgconfig/assimp.pc ]; then mv $(INSTALL_DIR)/lib/pkgconfig/assimp.pc $(INSTALL_DIR)/lib/pkgconfig/assimp_devel.pc ; fi
	if [ -e $(INSTALL_DIR)/include/assimp ]; then rm -fr $(INSTALL_DIR)/include/assimp_devel; mv $(INSTALL_DIR)/include/assimp $(INSTALL_DIR)/include/assimp_devel; fi

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm $(TARBALL)
	cd $(INSTALL_DIR) && rm -rf lib include bin
