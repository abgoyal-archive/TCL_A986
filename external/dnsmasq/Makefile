PREFIX = /usr/local
BINDIR = ${PREFIX}/sbin
MANDIR = ${PREFIX}/share/man
LOCALEDIR = ${PREFIX}/share/locale

SRC = src
PO = po
MAN = man

PKG_CONFIG = pkg-config
INSTALL = install
MSGMERGE = msgmerge
MSGFMT = msgfmt
XGETTEXT = xgettext

#################################################################

DNSMASQ_CFLAGS=`echo $(COPTS) | ../bld/pkg-wrapper HAVE_DBUS $(PKG_CONFIG) --cflags dbus-1` 
DNSMASQ_LIBS=  `echo $(COPTS) | ../bld/pkg-wrapper HAVE_DBUS $(PKG_CONFIG) --libs dbus-1` 
SUNOS_LIBS= `if uname | grep SunOS 2>&1 >/dev/null; then echo -lsocket -lnsl -lposix4; fi`

all :   dnsmasq

dnsmasq :
	@cd $(SRC) && $(MAKE) \
 DNSMASQ_CFLAGS="$(DNSMASQ_CFLAGS)" \
 DNSMASQ_LIBS="$(DNSMASQ_LIBS) $(SUNOS_LIBS)" \
 -f ../bld/Makefile dnsmasq 

clean :
	rm -f *~ $(SRC)/*.mo contrib/*/*~ */*~ $(SRC)/*.pot 
	rm -f $(SRC)/*.o $(SRC)/dnsmasq.a $(SRC)/dnsmasq core */core

install : all install-common

install-common :
	$(INSTALL) -d $(DESTDIR)$(BINDIR) -d $(DESTDIR)$(MANDIR)/man8
	$(INSTALL) -m 644 $(MAN)/dnsmasq.8 $(DESTDIR)$(MANDIR)/man8 
	$(INSTALL) -m 755 $(SRC)/dnsmasq $(DESTDIR)$(BINDIR)

all-i18n :
	@cd $(SRC) && $(MAKE) \
 I18N=-DLOCALEDIR='\"$(LOCALEDIR)\"' \
 DNSMASQ_CFLAGS="$(DNSMASQ_CFLAGS) `$(PKG_CONFIG) --cflags libidn`" \
 DNSMASQ_LIBS="$(DNSMASQ_LIBS) $(SUNOS_LIBS) `$(PKG_CONFIG) --libs libidn`"  \
 -f ../bld/Makefile dnsmasq 
	@cd $(PO); for f in *.po; do \
		cd ../$(SRC) && $(MAKE) \
 MSGMERGE=$(MSGMERGE) MSGFMT=$(MSGFMT) XGETTEXT=$(XGETTEXT) \
 -f ../bld/Makefile $${f%.po}.mo; \
	done

install-i18n : all-i18n install-common
	cd $(SRC); ../bld/install-mo $(DESTDIR)$(LOCALEDIR) $(INSTALL)
	cd $(MAN); ../bld/install-man $(DESTDIR)$(MANDIR) $(INSTALL)

merge :
	@cd $(SRC) && $(MAKE) XGETTEXT=$(XGETTEXT) -f ../bld/Makefile dnsmasq.pot
	@cd $(PO); for f in *.po; do \
		echo -n msgmerge $$f && $(MSGMERGE) --no-wrap -U $$f ../$(SRC)/dnsmasq.pot; \
	done


