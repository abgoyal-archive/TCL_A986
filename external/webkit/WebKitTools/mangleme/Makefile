CC = gcc
CFLAGS = -Wall -O3 -fomit-frame-pointer -funroll-loops
OUTDIR = ../../WebKitBuild/mangleme

all: $(OUTDIR)/mangle.cgi $(OUTDIR)/remangle.cgi

$(OUTDIR)/%.cgi: %.cgi.c
	if [ ! -d $(OUTDIR) ]; then mkdir -p $(OUTDIR); fi
	$(CC) $(CFLAGS) $< -o $(OUTDIR)/$*.cgi
