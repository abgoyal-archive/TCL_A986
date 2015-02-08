KERNELDIR = /usr/src/kernel-headers-2.4.27

CFLAGS = -D__KERNEL__ -DMODULE   -I$(KERNELDIR)/include -O2 -Wall -g



TARGET = nandemul2k.o

default: $(TARGET)

clean:
	rm -f $(TARGET)

$(TARGET): %.o: %.c
	gcc -c $(CFLAGS) $< -o $@

