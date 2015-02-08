DEBUG = 1

.autodepend

all:	initflex.exe flex.exe
	rm initflex.exe initflex.map

###############################################################################
#
# standard utilitities? ha.
#

CC	= bcc
CPP     = bcc

###############################################################################
#

MODEL	= l

!if $(DEBUG) == 1
!message Building with debug.
debugCompile = -v
debugLink = /v
!else
!message Building without debug.
debugCompile =
debugLink =
!endif

LOADER	= c0$(MODEL).obj
LIBS	= c$(MODEL).lib
LINKFLAGS = $(debugLink)

DATASEG	= -dc -Ff
SizeOPT	= -Os -G-
Defines =

COMMON	= -A -c -m$(MODEL) $(SizeOPT) $(DATASEG) $(Defines) $(debugCompile)
CFLAGS  = -o$@ $(COMMON)
CCFLAGS  = -o$@ $(COMMON) -Pcc

###############################################################################

.SUFFIXES:	.cc

.cc.obj:
	$(CPP) $(CCFLAGS) $<

.c.obj:
	$(CPP) $(CFLAGS) $<

###############################################################################
#
# source & object files
#

BASESRC = ccl.c dfa.c ecs.c gen.c main.c misc.c nfa.c parse.c \
	sym.c tblcmp.c yylex.c skel.c

INITSRC = $(BASESRC) initscan.c

INITOBJS = $(INITSRC:.c=.obj)

SRC = $(BASESRC) scan.c

OBJS = $(SRC:.c=.obj)

objects:	$(OBJS)
	@echo $(OBJS)

###############################################################################
#
# Executable
#

initflex.exe:      $(INITOBJS)
	tlink $(LINKFLAGS) @&&!
$(LOADER) $**
$&.exe

$(LIBS)
!

flex.exe:      $(OBJS)
	tlink $(LINKFLAGS) @&&!
$(LOADER) $**
$&.exe

$(LIBS)
!

# 
###############################################################################
#
# Lex files
#

FLEX	= .\initflex
FLEX_FLAGS = -ist

scan.c: scan.l
	$(FLEX) $(FLEX_FLAGS) scan.l >scan.tmp
	sed s,\"$(srcdir)/scan.l\",\"scan.l\", <scan.tmp >scan.c
	@rm scan.tmp

###############################################################################
#
# YACC files
#

YACC	= .\bison
YFLAGS  = -vdyl

parse.c: parse.y
	$(YACC) -ydl parse.y
	@sed "/extern char.*malloc/d" <y_tab.c >parse.c
	@rm -f y_tab.c
	@mv y_tab.h parse.h

###############################################################################
#
# cleanup
#

clean:
	-rm *.obj *.map initflex.exe

realclean:	clean
	-rm flex.exe

#
# end Makefile
#
###############################################################################
