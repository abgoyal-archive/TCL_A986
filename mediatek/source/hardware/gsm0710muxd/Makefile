CROSS_COMPILE=
#commands line variable. Extra compiler flags
EXTRA_CFLAGS=

CC:=gcc
CFLAGS:= $(EXTRA_CFLAGS) -Wall -O2
LDFLAGS:= -Wall -O2
LIBS:=-lpthread


objects = src/gsm0710muxd.o

gsm0710muxd : $(objects)
	$(CROSS_COMPILE)$(CC) $(LDFLAGS) $(LIBS) $^ -o $@ 


src/gsm0710muxd.o : src/gsm0710muxd.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -c $^ -o $@


.PHONY : clean
clean :
	-rm gsm0710muxd $(objects)












