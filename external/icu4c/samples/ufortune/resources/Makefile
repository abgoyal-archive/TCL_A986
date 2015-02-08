include ../../defs.mk

# No conventional target - this dir is resources only.
TARGET=
CLEANFILES += *.[co] *.lst $(RESNAME)_*.mak $(RESNAME).dat $(RESFILES) *.ao README*resources.txt $(RESNAME)_dat*

include ../fortunedefs.mk

# target file for resource bundle - this must be set, or 'make all' won't
# build any resources.
ifeq ($(RESMODE),dll)
RESTARGET=lib$(RESNAME).$(SO)
endif
ifeq ($(RESMODE),static)
RESTARGET=lib$(RESNAME).a
endif

# Resource files.  Add new ones for additional locales here.
# keep in sync with the file RESLIST
RESFILES=root.res es.res

# list of resource bundles  - keep in sync with RESFILES
RESLIST=res-file-list.txt

## Include standard rules 
include ../../rules.mk

# for installing the library
install: res-install

es.res: es.txt
	@echo "generating $@"
	$(GENRB) $(GENRBOPT) $^

