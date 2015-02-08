

#ifndef __LINUX_MTD_PHYSMAP__
#define __LINUX_MTD_PHYSMAP__

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct map_info;

struct physmap_flash_data {
	unsigned int		width;
	void			(*set_vpp)(struct map_info *, int);
	unsigned int		nr_parts;
	unsigned int		pfow_base;
	struct mtd_partition	*parts;
};

void physmap_configure(unsigned long addr, unsigned long size,
		int bankwidth, void (*set_vpp)(struct map_info *, int) );

#ifdef CONFIG_MTD_PARTITIONS

void physmap_set_partitions(struct mtd_partition *parts, int num_parts);

#endif /* defined(CONFIG_MTD_PARTITIONS) */

#endif /* __LINUX_MTD_PHYSMAP__ */
