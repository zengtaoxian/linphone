## Process this file with automake to produce Makefile.in

AM_CPPFLAGS=\
	-I$(top_srcdir) \
	-I$(top_srcdir)/coreapi \
	-I$(top_srcdir)/include 

COMMON_CFLAGS=\
	-DIN_LINPHONE \
	-D_ORTP_SOURCE \
	$(STRICT_OPTIONS) \
	$(ORTP_CFLAGS) \
	$(MEDIASTREAMER_CFLAGS) \
	$(VIDEO_CFLAGS) \
	$(READLINE_CFLAGS) \
	$(SQLITE3_CFLAGS) 

if BUILD_CONSOLE

bin_PROGRAMS=linphonec linphonecsh

if BUILD_WIN32
bin_PROGRAMS+=linphoned
endif

linphonec_SOURCES=linphonec.c linphonec.h commands.c
linphonec_CFLAGS=$(COMMON_CFLAGS) $(CONSOLE_FLAGS)
linphonec_LDADD=$(top_builddir)/coreapi/liblinphone.la \
		$(READLINE_LIBS)  \
		$(SQLITE3_LIBS) \
		$(X11_LIBS)

if BUILD_WIN32
#special build of linphonec to detach from the windows console
linphoned_SOURCES=$(linphonec_SOURCES)
linphoned_CFLAGS=$(COMMON_CFLAGS) $(GUI_FLAGS)
linphoned_LDADD=$(linphonec_LDADD)
endif


linphonecsh_SOURCES=shell.c
linphonecsh_CFLAGS=$(COMMON_CFLAGS) $(CONSOLE_FLAGS)
linphonecsh_LDADD=$(ORTP_LIBS)

endif


