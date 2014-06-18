all: installed
# ff
FILENAME = seq-sat-ffha.tar.bz2
TARBALL = build/$(FILENAME)
TARBALL_URL = "http://ipc.informatik.uni-freiburg.de/Planners?action=AttachFile&do=get&target=$(FILENAME)"
SOURCE_DIR = build/seq-sat-ffha
UNPACK_CMD = tar jxf
MD5SUM_DIR = $(CURDIR)
MD5SUM_FILE = $(MD5SUM_DIR)/$(FILENAME).md5sum
include $(shell rospack find mk)/download_unpack_build.mk

PATCH_DIR  =  $(CURDIR)
PATCH =
INSTALL_DIR = `rospack find ffha`
MAKE = make
#MAKE_ARGS = ADDONS=-DYY_SKIP_YYWRAP

$(SOURCE_DIR)/patched:$(SOURCE_DIR)/unpacked
	rm -f $(SOURCE_DIR)/ffha
	patch --forward -p0 $(SOURCE_DIR)/inst_pre.c < ${PATCH_DIR}/inst_pre.c.patch
	patch --forward -p0 $(SOURCE_DIR)/parse.c < ${PATCH_DIR}/parse.c.patch
	touch $(SOURCE_DIR)/patched

installed:$(SOURCE_DIR)/patched
	(cd $(SOURCE_DIR) && $(MAKE) clean && $(MAKE) $(MAKE_ARGS))
	mkdir -p bin
	cp $(SOURCE_DIR)/ffha bin/
	touch installed

clean:
	rm -rf build
	rm -rf bin
	rm -rf installed

wiped: Makefile	make wipe
	touch wiped

wipe: 	clean
	touch wiped
