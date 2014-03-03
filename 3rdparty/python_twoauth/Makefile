all: installed

TOP_DIR = $(shell pwd)
GIT_DIR = build/python-twoauth
GIT_URL = https://github.com/techno/python-twoauth
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	cd $(GIT_DIR) && python setup.py build
	cd $(GIT_DIR) && python setup.py install --prefix=$(TOP_DIR) --install-purelib=$(TOP_DIR)/src
	mkdir -p bin && cp $(GIT_DIR)/sample/*.py ./bin && sed -i s@'import twoauth'@'import roslib; roslib.load_manifest("python_twoauth"); import twoauth'@ bin/*.py && chmod a+x bin/*.py
	touch installed

clean:
	rm -rf installed bin lib src

wipe: clean
	rm -rf $(GIT_DIR)
