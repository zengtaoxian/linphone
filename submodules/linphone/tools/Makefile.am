## Process this file with automake to produce Makefile.in

AM_CPPFLAGS=\
	-I$(top_srcdir) \
	-I$(top_srcdir)/coreapi 

COMMON_CFLAGS=\
	-DIN_LINPHONE \
	$(ORTP_CFLAGS) \
	$(MEDIASTREAMER_CFLAGS) \
	$(STRICT_OPTIONS) \
	$(LIBXML2_CFLAGS) 

#-fpermissive to workaround a g++ bug on macos 32bit SDK.
AM_CXXFLAGS=$(LIBXML2_CFLAGS) $(STRICT_OPTIONS) -fpermissive

EXTRA_DIST=xml2lpc_jni.cc lpc2xml_jni.cc

if BUILD_TOOLS

lib_LTLIBRARIES=libxml2lpc.la liblpc2xml.la

libxml2lpc_la_SOURCES=\
	xml2lpc.c \
	xml2lpc.h

liblpc2xml_la_SOURCES=\
	lpc2xml.c \
	lpc2xml.h

libxml2lpc_la_CFLAGS=$(COMMON_CFLAGS)
libxml2lpc_la_LIBADD=\
	$(top_builddir)/coreapi/liblinphone.la \
	$(LIBXML2_LIBS) 

liblpc2xml_la_CFLAGS=$(COMMON_CFLAGS)
liblpc2xml_la_LIBADD=\
	$(top_builddir)/coreapi/liblinphone.la \
	$(LIBXML2_LIBS) 

libxml2lpc_la_LDFLAGS=-no-undefined
liblpc2xml_la_LDFLAGS=-no-undefined

bin_PROGRAMS=xml2lpc_test lpc2xml_test lp-gen-wrappers

xml2lpc_test_SOURCES=\
	xml2lpc_test.c

lpc2xml_test_SOURCES=\
	lpc2xml_test.c

xml2lpc_test_CFLAGS=$(COMMON_CFLAGS)
xml2lpc_test_LDADD=\
	$(top_builddir)/coreapi/liblinphone.la \
	libxml2lpc.la 

lpc2xml_test_CFLAGS=$(COMMON_CFLAGS)
lpc2xml_test_LDADD=\
	$(top_builddir)/coreapi/liblinphone.la \
	liblpc2xml.la

lp_gen_wrappers_SOURCES=genwrappers.cc \
			software-desc.cc software-desc.hh \
			generator.cc generator.hh

lp_gen_wrappers_LDADD= \
		$(LIBXML2_LIBS)

endif


