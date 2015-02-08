ifeq ($(wildcard /bin/s[h]),/bin/sh)
SHELL = /bin/sh
else
# will be used only with the next shell-test line, then overwritten
# by a configured-in value
SHELL = sh
endif

have-Makefile := $(shell test -f Makefile && echo yes)

# If the user runs GNU make but has not yet run ./configure,
# give them a diagnostic.
ifeq ($(have-Makefile),yes)

# Make tar archive easier to reproduce.
export TAR_OPTIONS = --owner=0 --group=0 --numeric-owner

include Makefile
include $(srcdir)/Makefile.cfg
include $(srcdir)/Makefile.maint

else

all:
	@echo There seems to be no Makefile in this directory.
	@echo "You must run ./configure before running \`make'."
	@exit 1

endif

# Tell version 3.79 and up of GNU make to not build goals in this
# directory in parallel.  This is necessary in case someone tries to
# build multiple targets on one command line.
.NOTPARALLEL:
