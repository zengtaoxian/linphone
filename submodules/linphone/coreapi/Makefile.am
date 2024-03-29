GITVERSION_FILE=liblinphone_gitversion.h
GITVERSION_FILE_TMP=liblinphone_gitversion.h.tmp
GITDESCRIBE=`cd $(top_srcdir) && git describe --always`
GIT_TAG=`cd $(top_srcdir) && git describe --abbrev=0`
GITREVISION=`cd $(top_srcdir) && git rev-parse HEAD`

ECHO=/bin/echo

SUBDIRS=. help

EXTRA_DIST=linphonecore_jni.cc $(GITVERSION_FILE)

BUILT_SOURCES=$(GITVERSION_FILE)

CLEANFILES=$(GITVERSION_FILE)

## Process this file with automake to produce Makefile.in
linphone_includedir=$(includedir)/linphone

linphone_include_HEADERS=linphonecore.h linphonefriend.h linphonepresence.h linphonecore_utils.h lpconfig.h sipsetup.h event.h

if BUILD_TUNNEL
linphone_include_HEADERS+=linphone_tunnel.h
endif

lib_LTLIBRARIES=liblinphone.la

liblinphone_la_SOURCES=\
	linphonecore.c linphonecore.h private.h\
	offeranswer.c offeranswer.h\
	sal.c \
	callbacks.c \
	misc.c  \
	address.c \
	enum.c enum.h \
	presence.c \
	proxy.c \
	friend.c \
	authentication.c \
	lpconfig.c lpconfig.h \
	chat.c \
	linphonecall.c \
	sipsetup.c sipsetup.h \
	siplogin.c \
	lsd.c linphonecore_utils.h \
	ec-calibrator.c \
	conference.c \
	message_storage.c \
	info.c \
	event.c event.h \
	$(GITVERSION_FILE)

if BUILD_UPNP
liblinphone_la_SOURCES+=upnp.c upnp.h
endif
	
liblinphone_la_SOURCES+=	bellesip_sal/sal_address_impl.c \
				bellesip_sal/sal_impl.c bellesip_sal/sal_impl.h \
				bellesip_sal/sal_op_impl.c \
				bellesip_sal/sal_op_call.c \
				bellesip_sal/sal_op_registration.c \
				bellesip_sal/sal_sdp.c  \
				bellesip_sal/sal_op_message.c \
				bellesip_sal/sal_op_presence.c \
				bellesip_sal/sal_op_publish.c \
				bellesip_sal/sal_op_call_transfer.c \
				bellesip_sal/sal_op_info.c \
				bellesip_sal/sal_op_events.c

if BUILD_WIZARD
liblinphone_la_SOURCES+=sipwizard.c 
endif

liblinphone_la_SOURCES+=linphone_tunnel_config.c
if BUILD_TUNNEL
liblinphone_la_SOURCES+=linphone_tunnel.cc TunnelManager.cc TunnelManager.hh 
else
liblinphone_la_SOURCES+=linphone_tunnel_stubs.c
endif



liblinphone_la_LDFLAGS= -version-info $(LIBLINPHONE_SO_VERSION) -no-undefined

liblinphone_la_LIBADD= \
		$(SIPSTACK_LIBS) \
		$(MEDIASTREAMER_LIBS) \
		$(ORTP_LIBS) $(OPENSSL_LIBS) \
		$(TUNNEL_LIBS) \
		$(LIBSOUP_LIBS) \
		$(SQLITE3_LIBS) \
		$(LIBXML2_LIBS) 


if ENABLE_TESTS
noinst_PROGRAMS=test_lsd test_ecc test_numbers

test_lsd_SOURCES=test_lsd.c

test_lsd_LDADD=liblinphone.la $(liblinphone_la_LIBADD)

test_ecc_SOURCES=test_ecc.c

test_ecc_LDADD=liblinphone.la $(liblinphone_la_LIBADD)

test_numbers_SOURCES=test_numbers.c

test_numbers_LDADD=liblinphone.la $(liblinphone_la_LIBADD)
endif

AM_CPPFLAGS=\
	-I$(top_srcdir) -I$(top_srcdir)/include

AM_CFLAGS=\
	$(STRICT_OPTIONS)  -DIN_LINPHONE \
	$(ORTP_CFLAGS) \
	$(MEDIASTREAMER_CFLAGS) \
	$(SIPSTACK_CFLAGS) \
	$(LIBSOUP_CFLAGS) \
	-DENABLE_TRACE 	\
	-DLOG_DOMAIN=\"LinphoneCore\" \
	$(IPV6_CFLAGS) \
	-DORTP_INET6 \
	$(VIDEO_CFLAGS) \
	$(TUNNEL_CFLAGS) \
	$(SQLITE3_CFLAGS) \
	$(LIBXML2_CFLAGS)

if BUILD_WIZARD
AM_CFLAGS+= -DBUILD_WIZARD 
endif

AM_CFLAGS+= -DUSE_BELLESIP

AM_CXXFLAGS=$(AM_CFLAGS)

make_gitversion_h:
	if test "$(GITDESCRIBE)" != "" ; then \
		if test "$(GIT_TAG)" != "$(PACKAGE_VERSION)" ; then \
			echo "*** PACKAGE_VERSION and git tag differ. Please put them identical."; \
			exit 1; \
		fi ; \
		$(ECHO) -n "#define LIBLINPHONE_GIT_VERSION \"$(GITDESCRIBE)\"" > $(GITVERSION_FILE_TMP) ; \
	elif test "$(GITREVISION)" != "" ; then \
		$(ECHO) -n "#define LIBLINPHONE_GIT_VERSION \"$(LINPHONE_VERSION)_$(GITREVISION)\"" > $(GITVERSION_FILE_TMP) ; \
	else \
		$(ECHO) -n "" > $(GITVERSION_FILE_TMP) ; \
	fi
	if test ! -f $(srcdir)/$(GITVERSION_FILE) ; then \
		cp -f $(GITVERSION_FILE_TMP) $(srcdir)/$(GITVERSION_FILE) ; \
	fi
	if test "`cat $(GITVERSION_FILE_TMP)`" != "`cat $(srcdir)/$(GITVERSION_FILE)`" ; then \
		cp -f $(GITVERSION_FILE_TMP) $(srcdir)/$(GITVERSION_FILE) ; \
	fi
	rm -f $(GITVERSION_FILE_TMP) ;

$(GITVERSION_FILE): make_gitversion_h
