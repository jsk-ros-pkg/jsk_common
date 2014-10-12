all: installed

REV=f33d3b65601f
TARBALL=${REV}.tar.gz
TARBALL_URL=http://hg.fast-downward.org/archive/${TARBALL}
SOURCE_DIR=build/downward
INITIAL_DIR=build/Fast-Downward-${REV}
UNPACK_CMD=tar xvzf
include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR)/src && ./build_all
	touch installed

clean:
	cd $(SOURCE_DIR)/src && ./cleanup
	rm -f installed

wipe: clean
	rm -rf $(HG_DIR)

