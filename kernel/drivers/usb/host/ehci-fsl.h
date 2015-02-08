
#ifndef _EHCI_FSL_H
#define _EHCI_FSL_H

/* offsets for the non-ehci registers in the FSL SOC USB controller */
#define FSL_SOC_USB_ULPIVP	0x170
#define FSL_SOC_USB_PORTSC1	0x184
#define PORT_PTS_MSK		(3<<30)
#define PORT_PTS_UTMI		(0<<30)
#define PORT_PTS_ULPI		(2<<30)
#define	PORT_PTS_SERIAL		(3<<30)
#define PORT_PTS_PTW		(1<<28)
#define FSL_SOC_USB_PORTSC2	0x188
#define FSL_SOC_USB_USBMODE	0x1a8
#define FSL_SOC_USB_SNOOP1	0x400	/* NOTE: big-endian */
#define FSL_SOC_USB_SNOOP2	0x404	/* NOTE: big-endian */
#define FSL_SOC_USB_AGECNTTHRSH	0x408	/* NOTE: big-endian */
#define FSL_SOC_USB_PRICTRL	0x40c	/* NOTE: big-endian */
#define FSL_SOC_USB_SICTRL	0x410	/* NOTE: big-endian */
#define FSL_SOC_USB_CTRL	0x500	/* NOTE: big-endian */
#define SNOOP_SIZE_2GB		0x1e
#endif				/* _EHCI_FSL_H */
