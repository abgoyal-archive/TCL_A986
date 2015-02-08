obj-y := config.o

extra-y := bootlogo.rh

$(obj)/bootlogo.rh: $(src)/bootlogo.h
	perl $(src)/../68328/bootlogo.pl < $(src)/bootlogo.h \
		> $(obj)/bootlogo.rh
