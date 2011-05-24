all: installed

INSTALL_DIR=`rospack find crlibm`
TARBALL_NAME = crlibm-1.0beta4
TARBALL = build/$(TARBALL_NAME).tar.gz
TARBALL_URL = http://lipforge.ens-lyon.fr/frs/download.php/162/$(TARBALL_NAME).tar.gz
SOURCE_DIR = build/$(TARBALL_NAME)
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(TARBALL_NAME).md5sum
TARBALL_PATCH=
BUILD_DIR = build/build
include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked CMakeLists.txt
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) ../..
	cd $(BUILD_DIR) && make && make install 
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-make -C $(BUILD_DIR) clean
	rm -rf $(BUILD_DIR)
	rm -rf installed

wipe: clean
	rm -rf build patched

wiped: Makefile
	make wipe
	touch wiped

.PHONY : clean download
