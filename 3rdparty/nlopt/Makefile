all: installed

TARBALL = build/nlopt-2.3.tar.gz
TARBALL_URL = http://ab-initio.mit.edu/nlopt/nlopt-2.3.tar.gz
SOURCE_DIR = build/nlopt-2.3
MD5SUM_FILE = nlopt-2.3.tar.gz.md5sum
UNPACK_CMD = tar xzf
DSTDIR = $(PWD)
MK_DIR = $(shell rospack find mk)
include $(MK_DIR)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	(cd $(SOURCE_DIR) && ./configure --enable-shared --with-cxx --prefix=$(DSTDIR) LIBS='-lstdc++' && make && make install)
	touch installed
clean:
	-rm -rf include lib share $(SOURCE_DIR) installed

wipe: clean
	-rm -rf build
