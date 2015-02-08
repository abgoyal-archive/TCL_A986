

#ifndef __LINUX_USB_M66592_H
#define __LINUX_USB_M66592_H

#define M66592_PLATDATA_XTAL_12MHZ	0x01
#define M66592_PLATDATA_XTAL_24MHZ	0x02
#define M66592_PLATDATA_XTAL_48MHZ	0x03

struct m66592_platdata {
	/* one = on chip controller, zero = external controller */
	unsigned	on_chip:1;

	/* one = big endian, zero = little endian */
	unsigned	endian:1;

	/* (external controller only) M66592_PLATDATA_XTAL_nnMHZ */
	unsigned	xtal:2;

	/* (external controller only) one = 3.3V, zero = 1.5V */
	unsigned	vif:1;

};

#endif /* __LINUX_USB_M66592_H */

