all: installed

REV=f33d3b65601f
TARBALL=Fast-Downward-${REV}.tar.gz
# http://hg.fast-downward.org/archive/
TARBALL_URL=https://github.com/jsk-ros-pkg/archives/raw/master/${TARBALL}
SOURCE_DIR=build/downward
INITIAL_DIR=build/Fast-Downward-${REV}
UNPACK_CMD=tar xvzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)/src && ./build_all DOWNWARD_BITWIDTH=native
	touch installed

clean:
	cd $(SOURCE_DIR)/src && ./cleanup
	rm -f installed

wipe: clean
	rm -rf $(HG_DIR)

