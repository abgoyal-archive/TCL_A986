

#ifndef __U_PHONET_H
#define __U_PHONET_H

#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>

int gphonet_setup(struct usb_gadget *gadget);
int phonet_bind_config(struct usb_configuration *c);
void gphonet_cleanup(void);

#endif /* __U_PHONET_H */
