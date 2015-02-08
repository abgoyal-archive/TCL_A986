obj-y := config.o commproc.o entry.o ints.o

extra-y := head.o

$(obj)/head.o: $(obj)/head-$(MODEL).o
	ln -sf head-$(MODEL).o $(obj)/head.o
