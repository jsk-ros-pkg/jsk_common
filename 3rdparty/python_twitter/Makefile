all: installed

TOP_DIR = $(shell pwd)
GIT_DIR = build/python-twitter
GIT_URL = https://github.com/bear/python-twitter
GIT_REVISION = v0.8.5
include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR) patched
	mkdir -p src; mkdir -p bin
	mkdir -p $(TOP_DIR)/build/lib/python2.7/site-packages/
	cd $(GIT_DIR) && pip install --user -r requirements.txt
	cd $(GIT_DIR) && python setup.py build
	cd $(GIT_DIR) && PYTHONPATH=$(TOP_DIR)/build/lib/python2.7/site-packages python setup.py install --prefix=$(TOP_DIR)/build #--install-purelib=$(TOP_DIR)/src
	-cp $(TOP_DIR)/build/lib/python2.7/site-packages/*.egg/* ./src/
	-cp -r $(TOP_DIR)/build/lib/python2.7/site-packages/*.egg/* ./src/
	cp $(GIT_DIR)/examples/*.py ./bin && sed -i s@'import twitter'@'import roslib; roslib.load_manifest("python_twitter"); import twitter'@ bin/*.py && sed -i s@'#!/usr/bin/python2.4'@'#!/usr/bin/env python'@ bin/*.py
	touch installed

clean:
	rm -rf installed bin lib src build

wipe: clean
	rm -rf $(GIT_DIR)
