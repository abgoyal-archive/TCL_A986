obj-y				:= cpu.o irq.o time.o gpio.o devices.o
obj-m				:=
obj-n				:=
obj-				:=

# PCI support is optional
obj-$(CONFIG_PCI)		+= pci.o

# LEDs
obj-$(CONFIG_LEDS)		+= leds.o

# Board-specific support
obj-$(CONFIG_MACH_KS8695)	+= board-micrel.o
obj-$(CONFIG_MACH_DSM320)	+= board-dsm320.o
obj-$(CONFIG_MACH_ACS5K)	+= board-acs5k.o
