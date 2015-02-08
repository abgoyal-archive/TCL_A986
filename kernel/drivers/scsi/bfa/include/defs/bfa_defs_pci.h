

#ifndef __BFA_DEFS_PCI_H__
#define __BFA_DEFS_PCI_H__

enum {
	BFA_PCI_VENDOR_ID_BROCADE	= 0x1657,
	BFA_PCI_DEVICE_ID_FC_8G2P	= 0x13,
	BFA_PCI_DEVICE_ID_FC_8G1P	= 0x17,
	BFA_PCI_DEVICE_ID_CT		= 0x14,
};

enum {
	BFA_PCI_FCOE_SSDEVICE_ID	= 0x14,
};

#define BFA_PCI_ACCESS_RANGES 1	/* Maximum number of device address ranges
				 * mapped through different BAR(s). */

#endif /* __BFA_DEFS_PCI_H__ */
