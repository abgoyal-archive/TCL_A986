obj-y := cpu.o setup.o serial.o octeon-platform.o octeon-irq.o csrc-octeon.o
obj-y += dma-octeon.o flash_setup.o
obj-y += octeon-memcpy.o

obj-$(CONFIG_SMP)                     += smp.o

EXTRA_CFLAGS += -Werror
