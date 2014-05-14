all: assimp_devel

INSTALL_DIR=`rospack find assimp_devel`
GIT_DIR = assimp_git
GIT_URL = git://github.com/assimp/assimp.git --depth=50 -b master
GIT_REVISION = f63cf6b5c835033cc3c3875f442e99c9763c40f1
PATCH_DIR = $(CURDIR)
GIT_PATCH = ${PATCH_DIR}/assimp_git.unit.patch ${PATCH_DIR}/assimp_git.technique.patch ${PATCH_DIR}/assimp_git.obj_export.patch ${PATCH_DIR}/assimp_devel.patch
MK_DIR    = $(shell rospack find mk)
include $(MK_DIR)/git_checkout.mk

.PHONY: assimp_devel

build: assimp_devel

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)
CPU_NUM=$(shell grep -c processor /proc/cpuinfo)
PARALLEL_JOB=$(shell if `expr $(CPU_NUM) \> 4 > /dev/null`;then echo 4; else echo ${CPU_NUM}; fi)
assimp_devel: $(GIT_DIR) patched
	cd $(GIT_DIR) && mkdir -p $(BUILDDIR) && cd $(BUILDDIR) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) .. && make -j$(PARALLEL_JOB) install
	if [ -e $(INSTALL_DIR)/lib/pkgconfig/assimp.pc ]; then mv $(INSTALL_DIR)/lib/pkgconfig/assimp.pc $(INSTALL_DIR)/lib/pkgconfig/assimp_devel.pc ; fi

	if [ -e $(INSTALL_DIR)/include/assimp ]; then rm -fr $(INSTALL_DIR)/include/assimp_devel; mv $(INSTALL_DIR)/include/assimp $(INSTALL_DIR)/include/assimp_devel; fi

clean:
	rm -rf patched

wipe: clean
	cd $(INSTALL_DIR) && rm -rf $(GIT_DIR) include lib share patched
