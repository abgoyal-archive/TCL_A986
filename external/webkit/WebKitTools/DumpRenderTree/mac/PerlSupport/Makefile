CONFIGURATION_BUILD_DIR ?= .
OUTPUT_DIR=$(CONFIGURATION_BUILD_DIR)

WRAPPER_DIR=$(OUTPUT_DIR)/DerivedSources/DumpRenderTree
WRAPPER=$(WRAPPER_DIR)/DumpRenderTreeSupport_wrap.c
PERL_MODULE=$(OUTPUT_DIR)/DumpRenderTreeSupport.pm
DYLIB=$(OUTPUT_DIR)/DumpRenderTreeSupport.dylib
DUMPRENDERTREE=$(OUTPUT_DIR)/DumpRenderTree
PERL=/usr/bin/perl

OSX_VERSION=$(shell sw_vers -productVersion | cut -d. -f 2)

ifneq "$(OSX_VERSION)" "4"


SWIG=/usr/bin/swig

all: $(DYLIB) $(PERL_MODULE)

$(WRAPPER) $(PERL_MODULE): DumpRenderTreeSupport.c $(DUMPRENDERTREE)
	mkdir -p $(WRAPPER_DIR)
	$(SWIG) -o $(WRAPPER) -outdir $(OUTPUT_DIR) -perl -module DumpRenderTreeSupport $<


else


all: $(DYLIB) $(PERL_MODULE)

$(WRAPPER): DumpRenderTreeSupport_wrapTiger.c $(DUMPRENDERTREE)
	mkdir -p $(WRAPPER_DIR)
	cp DumpRenderTreeSupport_wrapTiger.c $(WRAPPER)

$(PERL_MODULE): DumpRenderTreeSupportTiger.pm $(DUMPRENDERTREE)
	cp DumpRenderTreeSupportTiger.pm $(PERL_MODULE)


endif

$(DYLIB): DumpRenderTreeSupport.c $(WRAPPER)
	gcc -g -dynamiclib -o $(DYLIB) `$(PERL) -MExtUtils::Embed -eperl_inc` `$(PERL) -MExtUtils::Embed -eldopts` $^

clean:
	rm -f $(WRAPPER) $(PERL_MODULE) $(DYLIB)

installhdrs installsrc install:
