

#ifndef __RT3090_H__
#define __RT3090_H__

#ifdef RT3090

#ifndef RTMP_PCI_SUPPORT
#error "For RT3090, you should define the compile flag -DRTMP_PCI_SUPPORT"
#endif

#ifndef RTMP_MAC_PCI
#error "For RT3090, you should define the compile flag -DRTMP_MAC_PCI"
#endif

#ifndef RTMP_RF_RW_SUPPORT
#error "For RT3090, you should define the compile flag -DRTMP_RF_RW_SUPPORT"
#endif

#ifndef RT30xx
#error "For RT3090, you should define the compile flag -DRT30xx"
#endif

#define PCIE_PS_SUPPORT

#include "mac_pci.h"
#include "rt30xx.h"

/* */
/* Device ID & Vendor ID, these values should match EEPROM value */
/* */
#define NIC3090_PCIe_DEVICE_ID  0x3090	/* 1T/1R miniCard */
#define NIC3091_PCIe_DEVICE_ID  0x3091	/* 1T/2R miniCard */
#define NIC3092_PCIe_DEVICE_ID  0x3092	/* 2T/2R miniCard */

#endif /* RT3090 // */

#endif /*__RT3090_H__ // */
