


#ifndef R8192_PM_H
#define R8192_PM_H

#include <linux/types.h>
#include <linux/usb.h>

int rtl8192U_save_tate (struct pci_dev *dev, u32 state);
int rtl8192U_suspend(struct usb_interface *intf, pm_message_t state);
int rtl8192U_resume (struct usb_interface *intf);
int rtl8192U_enable_wake (struct pci_dev *dev, u32 state, int enable);

#endif //R8192U_PM_H
