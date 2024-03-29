#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])

AC_INIT([belle-sip],[1.2.4],[jehan.monnier@linphone.org])

AC_CONFIG_SRCDIR([src/belle_sip_utils.c])

AC_CONFIG_HEADERS([config.h])

AC_CANONICAL_SYSTEM

dnl initialize pkg-config so that we can use it within if else fi statements.
PKG_PROG_PKG_CONFIG()

AM_INIT_AUTOMAKE
AM_SILENT_RULES(yes)

# Checks for programs.
AC_PROG_CC
dnl because of tunnel library wrapper, C++ is required.
AC_PROG_CXX
AM_PROG_CC_C_O

dnl AC_PROG_CC_C99
LT_INIT(win32-dll)

dnl Workaround for mingw, whose compiler doesn't check in /usr/include ...
case "$target_os" in
	*mingw*)
		if test "$cross_compiling" = "yes"; then
			CPPFLAGS="$CPPFLAGS -DUSE_FIXED_NAMESERVERS"
		else
			if test "$prefix" = "/usr" ; then
				CPPFLAGS="$CPPFLAGS -I/usr/include"
				LDFLAGS="$LDFLAGS -L/usr/lib"
			fi
		fi
	;;
esac

if test -f /etc/debian_version ; then
	use_deb=true;
else
	use_rpm=true;
fi

AC_ARG_ENABLE(debug,
		[  --enable-debug    Turn on debug mode (default=no)],
		[case "${enableval}" in
		yes) debug=true ;;
		no)  debug=false ;;
		*) AC_MSG_ERROR(bad value ${enableval} for --enable-debug) ;;
		esac],[debug=false])

CFLAGS="$CFLAGS -fms-extensions"
if test "$debug" = "no" ; then
	CFLAGS="$CFLAGS -g -O2"
else
	CFLAGS="$CFLAGS -g"
fi

STRICT_OPTIONS=" -Wall -Werror -Wno-unknown-pragmas -Wno-error=unknown-pragmas"
dnl because of antlr3 we must accept a few warnings...
dnl more portable for the moment
       LESS_STRICT_OPTIONS= 
dnl       LESS_STRICT_OPTIONS="$STRICT_OPTIONS -Wno-error=unused-function \
dnl        -Wno-error=uninitialized -Wno-error=unused-variable \
dnl        -Wno-error=unused-but-set-variable "

AC_SUBST(STRICT_OPTIONS)
AC_SUBST(LESS_STRICT_OPTIONS)

# Checks for libraries.

# Checks for header files.
AC_ARG_WITH( antlr,
      [  --with-antlr      Set prefix where libantlr3c can be found (ex:/usr or /usr/local)[default=PREFIX] ],
      [ antlr_prefix=${withval}],[ antlr_prefix=${prefix} ])

found_antlr3=no
if test "$antlr_prefix" != "NONE" ; then
	ANTLR_CFLAGS="-I${antlr_prefix}/include"
	ANTLR_LIBS="-L${antlr_prefix}/lib"
fi

ANTLR_LIBS="$ANTLR_LIBS -lantlr3c"

dnl check antlr headers
CPPFLAGS_save=$CPPFLAGS
CPPFLAGS="$CPPFLAGS $ANTLR_CFLAGS"
AC_CHECK_HEADERS([antlr3.h], [found_antlr3=yes])
AC_CHECK_DECL(	[antlr3StringStreamNew]
		,[AC_DEFINE(HAVE_ANTLR_STRING_STREAM_NEW,1,[Defined when antlr 3.4 api is detected])]
		,[foo=bar]
		,[#include <antlr3.h>])
AC_CHECK_DECL(	[res_ndestroy]
		,[AC_DEFINE(HAVE_RESINIT,1,[Defined when res_ninit api is available])  LIBS="$LIBS -lresolv"]
		,[foo=bar]
		,[#include <resolv.h>])
				
CPPFLAGS=$CPPFLAGS_save

if test "$found_antlr3" != "yes" ; then
	AC_MSG_ERROR([Could not find antlr3 development files. Please install antlr3 version > 3.2 (libantlr3c-dev on debian/ubuntu systems)])
	ANTLR_CFLAGS=
	ANTLR_LIBS=
fi
AC_PATH_PROG([ANTLR],[antlr3],[no],[$antlr_prefix/bin /usr/bin])

if test $ANTLR = "no" ; then
antlr_java_prefixes="$antlr_prefix/share/java /usr/local/share/java /usr/share/java /opt/local/share/java"

	for antlr_java_prefix in $antlr_java_prefixes
	do
		antlr_jar=$antlr_java_prefix/antlr.jar
		if test -f $antlr_jar ; then
			break
		else
			antlr_jar=no
		fi
		antlr_jar=$antlr_java_prefix/antlr3.jar
		if test -f $antlr_jar ; then
			break
		else
			antlr_jar=no
		fi
	done
	if test $antlr_jar = "no" ; then
		AC_MSG_ERROR([Could not find antlr.jar. Please install antlr3 ])
	fi
	AC_PATH_PROG([JAVA],[java],[no])
	if test $JAVA = "no" ; then
		AC_MSG_ERROR([Could not find java prog. Please install java ])
	else	
		ANTLR="$JAVA -jar $antlr_jar"
	fi
fi
AC_SUBST(ANTLR_CFLAGS)
AC_SUBST(ANTLR_LIBS)

AC_ARG_ENABLE(	tls,
		[  --enable-tls		Enable TLS support (default=yes)],
		[case "${enableval}" in
		yes) use_tls=true ;;
		no)  use_tls=false ;;
		*) AC_MSG_ERROR(bad value ${enableval} for --enable-tls) ;;
		esac],[use_tls=true])

AC_ARG_WITH( polarssl,
      [  --with-polarssl      Set prefix where polarssl can be found (ex:/usr, /usr/local)[default=PREFIX] ],
      [ polarssl_prefix=${withval}],[ polarssl_prefix=${prefix} ])

if test "$polarssl_prefix" != "NONE" ; then
	POLARSSL_CFLAGS="-I${polarssl_prefix}/include"
	POLARSSL_LIBS="-L${polarssl_prefix}/lib"
fi

POLARSSL_LIBS="$POLARSSL_LIBS -lpolarssl"
found_polarssl=no

if test "$use_tls" = "true" ; then
	CPPFLAGS_save=$CPPFLAGS
	LIBS_save=$LIBS

	CPPFLAGS="$CPPFLAGS $POLARSSL_CFLAGS"
	LIBS="$LIBS $POLARSSL_LIBS"
	AC_CHECK_HEADERS(polarssl/ssl.h,
		[
			AC_CHECK_LIB(polarssl,x509parse_crtpath,[
				found_polarssl=yes
				POLARSSL_LIBS=" -lpolarssl"
			])
			
	])
	CPPFLAGS=$CPPFLAGS_save
	LIBS=$LIBS_save
fi

AM_CONDITIONAL([BUILD_TLS], [test "x$found_polarssl" = "xyes"])
TLS_CFLAGS=""
TLS_LIBS=""
TLS_PC=""
if test "x$found_polarssl" = "xyes" ; then
	AC_DEFINE(HAVE_POLARSSL,1,[Defined when polarssl api is available])
	TLS_CFLAGS=$POLARSSL_CFLAGS
	TLS_LIBS=$POLARSSL_LIBS
fi
AC_SUBST(TLS_CFLAGS)
AC_SUBST(TLS_LIBS)	
AC_SUBST(TLS_PC)

AC_ARG_ENABLE(tunnel,
	[AS_HELP_STRING([--enable-tunnel], [Enable tunnel support (default=no)])])
if test "$enable_tunnel" = "yes" ; then
	PKG_CHECK_MODULES(TUNNEL, tunnel, [found_tunnel=yes], [found_tunnel=no])
	if test "$found_tunnel" = "yes" ; then
		AC_DEFINE(HAVE_TUNNEL, 1, [Defined when tunnel is enabled and available])
	fi
fi
AM_CONDITIONAL(BUILD_TUNNEL, test "$found_tunnel" = "yes")

PKG_CHECK_MODULES(CUNIT, cunit, [found_cunit=yes],[found_cunit=no])

if test "$found_cunit" = "no" ; then
	AC_CHECK_HEADERS(CUnit/CUnit.h,
		[
			AC_CHECK_LIB(cunit,CU_add_suite,[
				found_cunit=yes
				CUNIT_LIBS+=" -lcunit"
			])
			
		])
fi
case "$target_os" in
	*darwin*)
		#hack for macport
		CUNIT_LIBS+=" -lncurses"
	;;
	*mingw*)
		LIBBELLESIP_CFLAGS="-DBELLESIP_EXPORTS"
		CPPFLAGS="$CPPFLAGS -D_WIN32_WINNT=0x0501"
		LIBS="$LIBS -lws2_32 -liphlpapi"
		LDFLAGS="$LDFLAGS -Wl,--export-all-symbols"
	;;
esac
AC_SUBST(LIBBELLESIP_CFLAGS)

if test "$found_cunit" = "no" ; then
	AC_MSG_WARN([Could not find cunit framework, tests are not compiled.])
else 
	AC_CHECK_LIB(cunit,CU_get_suite,[
					AC_DEFINE(HAVE_CU_GET_SUITE,1,[defined when CU_get_suite is available])
					],[foo=bar],[$CUNIT_LIBS])
	AC_CHECK_LIB(cunit,CU_curses_run_tests,[
					AC_DEFINE(HAVE_CU_CURSES,1,[defined when CU_curses_run_tests is available])
					],[foo=bar],[$CUNIT_LIBS])
fi
AC_ARG_ENABLE(tests,
        [AS_HELP_STRING([--disable-tests], [Disable compilation of tests])],
        [case "${enableval}" in
                yes)    tests_enabled=true ;;
                no)     tests_enabled=false ;;
                *)      AC_MSG_ERROR(bad value ${enableval} for --disable-tests) ;;
        esac],
        [tests_enabled=yes]
)
AM_CONDITIONAL(ENABLE_TESTS, test x$tests_enabled = xyes && test x$found_cunit = xyes)

# Checks for typedefs, structures, and compiler characteristics.


# Checks for library functions.
AC_CHECK_LIB(rt, clock_gettime)
AC_CHECK_LIB(dl, dlopen)


AC_CONFIG_FILES(
[
Makefile
include/Makefile
include/belle-sip/Makefile
src/Makefile
tester/Makefile
belle-sip.pc
belle-sip.spec
])
AC_OUTPUT
