

#ifndef __RTMP_IFACE_H__
#define __RTMP_IFACE_H__

#ifdef RTMP_PCI_SUPPORT
#include "iface/rtmp_pci.h"
#endif /* RTMP_PCI_SUPPORT // */
#ifdef RTMP_USB_SUPPORT
#include "iface/rtmp_usb.h"
#endif /* RTMP_USB_SUPPORT // */

struct rt_inf_pci_config {
	unsigned long CSRBaseAddress;	/* PCI MMIO Base Address, all access will use */
	unsigned int irq_num;
};

struct rt_inf_usb_config {
	u8 BulkInEpAddr;	/* bulk-in endpoint address */
	u8 BulkOutEpAddr[6];	/* bulk-out endpoint address */
};

struct rt_inf_rbus_config {
	unsigned long csr_addr;
	unsigned int irq;
};

typedef enum _RTMP_INF_TYPE_ {
	RTMP_DEV_INF_UNKNOWN = 0,
	RTMP_DEV_INF_PCI = 1,
	RTMP_DEV_INF_USB = 2,
	RTMP_DEV_INF_RBUS = 4,
} RTMP_INF_TYPE;

typedef union _RTMP_INF_CONFIG_ {
	struct rt_inf_pci_config pciConfig;
	struct rt_inf_usb_config usbConfig;
	struct rt_inf_rbus_config rbusConfig;
} RTMP_INF_CONFIG;

#endif /* __RTMP_IFACE_H__ // */
