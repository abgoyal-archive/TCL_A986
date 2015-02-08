
#ifndef ASMARM_MACH_FLASH_H
#define ASMARM_MACH_FLASH_H

struct mtd_partition;
struct mtd_info;

struct flash_platform_data {
	const char	*map_name;
	const char	*name;
	unsigned int	width;
	int		(*init)(void);
	void		(*exit)(void);
	void		(*set_vpp)(int on);
	void		(*mmcontrol)(struct mtd_info *mtd, int sync_read);
	struct mtd_partition *parts;
	unsigned int	nr_parts;
};

#endif
