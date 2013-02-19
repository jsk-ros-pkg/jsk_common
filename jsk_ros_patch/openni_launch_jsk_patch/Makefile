include $(shell rospack find mk)/cmake.mk

all: installed

HG_DIR = build/openni_launch
HG_URL = http://kforge.ros.org/openni/openni_launch
# HG_REVISION = openni_launch-0.3.4
HG_BRANCH = fuerte
HG_PATCH = openni_zoom_launch.patch
include $(shell rospack find mk)/hg_checkout.mk

$(HG_DIR):
	mkdir -p `dirname $(HG_DIR)`; hg clone $(HG_URL) $(HG_DIR)
	cd $(HG_DIR) && hg update $(HG_BRANCH) && hg update $(HG_REVISION)
	touch rospack_nosubdirs

installed: $(HG_DIR) patched
	mkdir -p `dirname $(HG_DIR)`
	cp -r `pwd`/$(HG_DIR)/launch .
	touch installed

clean:
	rm -rf installed patched launch

wipe: clean
	rm -rf $(HG_DIR)
