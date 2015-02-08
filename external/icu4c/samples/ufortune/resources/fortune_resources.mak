RESFILES= root.res es.res 

#
#  ICUDIR   the location of ICU, used to locate the tools for
#           compiling and packaging resources.
#
ICUDIR=..\..\..\..

#
#  The directory that contains the tools.
#
!IF "$(CFG)" == "x64\Release" || "$(CFG)" == "x64\Debug"
BIN=bin64
!ELSE
BIN=bin
!ENDIF

#
#  File name extensions for inference rule matching.
#    clear out the built-in ones (for .c and the like), and add
#    the definition for .txt to .res.
#
.SUFFIXES : .txt

#
#  Inference rule, for compiling a .txt file into a .res file.
#  -t fools make into thinking there are files such as es.res, etc
#
.txt.res:
	$(ICUDIR)\$(BIN)\genrb -d . $*.txt

#
#  all - nmake starts here by default
#
all: fortune_resources.dll

fortune_resources.dll: $(RESFILES)
	$(ICUDIR)\$(BIN)\pkgdata --name fortune_resources -v --mode dll -d . res-file-list.txt

