# $Id: Makefile.am 6752 2016-03-09 18:27:04Z albert $

CCID_BUNDLE = $(bundle)
CCID_LIB = libspringcard_ccid_tcp.$(DYN_LIB_EXT)

CCID_VERSION=CCID_VERSION=`$(srcdir)/convert_version.pl $(PACKAGE_VERSION)`


COMMON = ccid.c \
	ccid.h \
	ccid_ifdhandler.h \
	commands.c \
	commands.h \
	debug.h \
	defs.h \
	ifdhandler.c \
	utils.c \
	utils.h

NETWORK = ccid_network.c ccid_network.h

T1 = towitoko/atr.c \
	towitoko/atr.h \
	towitoko/defines.h \
	towitoko/pps.c \
	towitoko/pps.h \
	openct/buffer.c \
	openct/buffer.h \
	openct/checksum.c \
	openct/checksum.h \
	openct/proto-t1.c \
	openct/proto-t1.h
TOKEN_PARSER = tokenparser.l parser.h \
	strlcpy.c \
	misc.h \
	strlcpycat.h \
	simclist.c \
	simclist.h

if WITHOUT_PCSC
PROVIDED_BY_PCSC = debug.c
endif

#libspringcardccidtcp_la_SOURCES = $(COMMON) $(USB) $(ETHERNET) $(TOKEN_PARSER) $(PROVIDED_BY_PCSC) $(T1)
#libspringcardccidtcp_la_LIBADD = $(LEXLIB) $(LIBUSB_LIBS) $(PTHREAD_LIBS)
#libspringcardccidtcp_la_CFLAGS = $(PCSC_CFLAGS) $(LIBUSB_CFLAGS) $(PTHREAD_CFLAGS) \
#	$(SYMBOL_VISIBILITY) -D$(CCID_VERSION) -DSIMCLIST_NO_DUMPRESTORE
#libspringcardccidtcp_la_LDFLAGS = -avoid-version

#parse_SOURCES = parse.c debug.c ccid_usb.c $(TOKEN_PARSER)
#parse_LDADD = $(LIBUSB_LIBS)
#parse_CFLAGS = $(PCSC_CFLAGS) $(LIBUSB_CFLAGS) -DSIMCLIST_NO_DUMPRESTORE
lib_LTLIBRARIES = 
lib_LTLIBRARIES += libspringcard_ccid_tcp.la
libspringcard_ccid_tcp_la_SOURCES = $(COMMON) $(NETWORK) $(TOKEN_PARSER) $(PROVIDED_BY_PCSC) $(T1)
libspringcard_ccid_tcp_la_LIBADD = $(LEXLIB) $(LIBUSB_LIBS) $(PTHREAD_LIBS)
libspringcard_ccid_tcp_la_CFLAGS = $(PCSC_CFLAGS) $(LIBUSB_CFLAGS) $(PTHREAD_CFLAGS) \
	$(SYMBOL_VISIBILITY) -D$(CCID_VERSION) -DSIMCLIST_NO_DUMPRESTORE
libspringcard_ccid_tcp_la_LDFLAGS = -avoid-version

#parse_SOURCES = parse.c debug.c ccid_network.c $(TOKEN_PARSER)
#parse_LDADD = $(LIBUSB_LIBS)
#parse_CFLAGS = $(PCSC_CFLAGS) $(LIBUSB_CFLAGS) -DSIMCLIST_NO_DUMPRESTORE

EXTRA_DIST = Info.plist.src create_Info_plist.pl reader.conf.in \
	towitoko/COPYING towitoko/README openct/LICENSE \
	convert_version.pl 92_pcscd_springcardccidtcp.rules

# We can't use install-exec-local since we want to overwrite the install
# rule. We do not want to _add_ files to install
#install: $(LIBS_TO_INSTALL)

INSTALL_UDEV_RULE_FILE=@/bin/echo -e "\n\33[01;31m***************\n" ; echo "copy the src/92_pcscd_springcardccidtcp.rules file in udev directory (/etc/udev/rules.d/)" ; /bin/echo -e "\n***************\n\33[0m"

Info.plist: Info.plist.src $(srcdir)/../readers/supported_readers.txt
	$(srcdir)/create_Info_plist.pl $(srcdir)/../readers/supported_readers.txt $(srcdir)/Info.plist.src --target=$(CCID_LIB) --version=$(VERSION) $(NOCLASS) > Info.plist

DISTCLEANFILES = tokenparser.c Info.plist

install: libspringcard_ccid_tcp.la Info.plist
	$(mkinstalldirs) $(DESTDIR)$(usbdropdir)/$(CCID_BUNDLE)/Contents/$(BUNDLE_HOST)/
	cp Info.plist $(DESTDIR)$(usbdropdir)/$(CCID_BUNDLE)/Contents/
	cp .libs/$(CCID_LIB) $(DESTDIR)$(usbdropdir)/$(CCID_BUNDLE)/Contents/$(BUNDLE_HOST)/$(CCID_LIB)
#	$(INSTALL_UDEV_RULE_FILE)


# do not uninstall the serial driver by default
# use explicitely 'make uninstall_ccidtwin'

uninstall:
	rm -rf $(DESTDIR)$(usbdropdir)/$(CCID_BUNDLE)



