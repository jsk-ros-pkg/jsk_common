#override datadir=$(FLOWDESIGNER_DATA)
override datadir=${prefix}/lib/flowdesigner/toolbox/hark-jsk
#override libdir=$(FLOWDESIGNER_DATA)
override libdir=${prefix}/lib/flowdesigner/toolbox/hark-jsk

lib_LTLIBRARIES = libhark-jsk.la 

install-data-local:
	mkdir -p $(DESTDIR)$(datadir)
	(if [ -f $(DESTDIR)$(libdir)/lib$(PACKAGE) ]; then cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE) $(PACKAGE).tlb; else cd $(DESTDIR)$(libdir); rm -f $(PACKAGE).tlb; ln -s lib$(PACKAGE).so $(PACKAGE).tlb; fi)
	(info2def.pl $(libhark_jsk_la_SOURCES) > $(DESTDIR)$(datadir)/hark-jsk.def)

libhark_jsk_la_SOURCES = \
	MMSE_STSA.cc \
	SaveFeaturesAudioDB.cc \
	SaveSpectrumFile.cc \
	SpectrumFile.cc \
	SpectrumSubtraction.cc \
	VERSION.cc \
	RosJskHarkMsgsPublisher.cc \
	RosNodeGeneratorForJSK.cc

libhark_jsk_la_CXXFLAGS = -O3 -march=core2 -mtune=core2 -mmmx -msse -msse2 -msse3 -mfpmath=sse @GTK_CFLAGS@

libhark_jsk_la_LDFLAGS = -release $(LT_RELEASE) \
	`flowdesigner-config --libs config libflow audio_blocks` \
	`pkg-config libharkio --libs` \
	`pkg-config --libs FlowDesigner`

INCLUDES = `flowdesigner-config --cflags config libflow audio_blocks` \
	-I ../include \
	`pkg-config --cflags FlowDesigner`

libhark_jsk_la_LIBADD = -lboost_thread-mt
