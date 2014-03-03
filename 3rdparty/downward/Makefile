all: installed

HG_DIR=downward
HG_URL=http://hg.fast-downward.org
include $(shell rospack find mk)/hg_checkout.mk

installed: $(HG_DIR)
	cd $(HG_DIR)/src && ./build_all
	touch installed

clean:
	cd $(HG_DIR)/src && ./cleanup
	rm -f installed

wipe: clean
	rm -rf $(HG_DIR)