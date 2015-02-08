

#ifndef _AHCI_PLATFORM_H
#define _AHCI_PLATFORM_H

struct device;
struct ata_port_info;

struct ahci_platform_data {
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
	const struct ata_port_info *ata_port_info;
	unsigned int force_port_map;
	unsigned int mask_port_map;
};

#endif /* _AHCI_PLATFORM_H */
