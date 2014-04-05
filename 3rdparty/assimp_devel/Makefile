all: assimp_devel

INSTALL_DIR=`rospack find assimp_devel`
GIT_DIR = assimp_git
#GIT_URL = https://github.com/YoheiKakiuchi/assimp.git
#GIT_REVISION = jsk-trunk ## fixed bug of Ply parse
GIT_URL = git://github.com/assimp/assimp.git --depth=1
GIT_REVISION = master
GIT_PATCH = assimp_git.unit.patch assimp_git.technique.patch assimp_git.obj_export.patch
include $(shell rospack find mk)/git_checkout.mk

.PHONY: assimp_devel

build: assimp_devel

BOOST_INCLUDE_DIRS=$(shell rosboost-cfg --include_dirs)
BOOST_LIBRARY_DIRS=$(shell rosboost-cfg --lib_dirs)

BUILDDIR=$(shell if [ $(DEBUG) ]; then echo builddebug; else echo build; fi)
CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)
CPU_NUM=$(shell grep -c processor /proc/cpuinfo)
PARALLEL_JOB=$(shell if `expr $(CPU_NUM) \> 12 > /dev/null`;then echo 12; else echo ${CPU_NUM}; fi)
assimp_devel: $(GIT_DIR) patched
	cd $(GIT_DIR) && mkdir -p $(BUILDDIR) && cd $(BUILDDIR) && BOOST_INCLUDEDIR=$(BOOST_INCLUDE_DIRS) BOOST_LIBRARYDIR=$(BOOST_LIBRARY_DIRS) cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) .. && make -j$(PARALLEL_JOB) install
#	if [ -e $(INSTALL_DIR)/include/assimp3 ]; then rm -rf $(INSTALL_DIR)/include/assimp3; fi
#	mv $(INSTALL_DIR)/include/assimp $(INSTALL_DIR)/include/assimp3
#	mv $(INSTALL_DIR)/bin/assimp $(INSTALL_DIR)/bin/assimp_cmd
#	ln -sf $(INSTALL_DIR)/lib/libassimp.so $(INSTALL_DIR)/lib/libassimp3.so

clean:
	rm -rf patched

wipe: clean
	cd $(INSTALL_DIR) && rm -rf $(GIT_DIR) include lib share patched
