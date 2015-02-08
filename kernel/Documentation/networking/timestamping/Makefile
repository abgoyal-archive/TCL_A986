obj- := dummy.o

# List of programs to build
hostprogs-y := timestamping

# Tell kbuild to always build the programs
always := $(hostprogs-y)

HOSTCFLAGS_timestamping.o += -I$(objtree)/usr/include

clean:
	rm -f timestamping
