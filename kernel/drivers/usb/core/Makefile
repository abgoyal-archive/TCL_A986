usbcore-objs	:= usb.o hub.o hcd.o urb.o message.o driver.o \
			config.o file.o buffer.o sysfs.o endpoint.o \
			devio.o notify.o generic.o quirks.o devices.o

ifeq ($(CONFIG_PCI),y)
	usbcore-objs	+= hcd-pci.o
endif

ifeq ($(CONFIG_USB_DEVICEFS),y)
	usbcore-objs	+= inode.o
endif

obj-$(CONFIG_USB)	+= usbcore.o

ifeq ($(CONFIG_USB_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
