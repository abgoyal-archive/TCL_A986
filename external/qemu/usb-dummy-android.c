

#include "qemu-common.h"
#include "qemu-timer.h"
#include "monitor.h"
#include "hw/usb.h"

USBDevice *usb_host_device_open(const char *devname)
{
	return NULL;
}

int usb_host_device_close(const char *devname)
{
	return 0;
}
 
void usb_host_info(Monitor *mon)
{
    monitor_printf(mon, "   No devices\n");
}
