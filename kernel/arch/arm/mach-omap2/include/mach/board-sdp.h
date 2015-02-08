
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct flash_partitions {
	struct mtd_partition *parts;
	int nr_parts;
};

extern void sdp_flash_init(struct flash_partitions []);
