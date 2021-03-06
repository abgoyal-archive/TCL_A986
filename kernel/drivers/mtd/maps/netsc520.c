

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>



static struct mtd_partition partition_info[]={
    {
	    .name = "NetSc520 boot kernel",
	    .offset = 0,
	    .size = 0xc0000
    },
    {
	    .name = "NetSc520 Low BIOS",
	    .offset = 0xc0000,
	    .size = 0x40000
    },
    {
	    .name = "NetSc520 file system",
	    .offset = 0x100000,
	    .size = 0xe80000
    },
    {
	    .name = "NetSc520 High BIOS",
	    .offset = 0xf80000,
	    .size = 0x80000
    },
};
#define NUM_PARTITIONS ARRAY_SIZE(partition_info)

#define WINDOW_SIZE	0x00100000
#define WINDOW_ADDR	0x00200000

static struct map_info netsc520_map = {
	.name = "netsc520 Flash Bank",
	.size = WINDOW_SIZE,
	.bankwidth = 4,
	.phys = WINDOW_ADDR,
};

#define NUM_FLASH_BANKS	ARRAY_SIZE(netsc520_map)

static struct mtd_info *mymtd;

static int __init init_netsc520(void)
{
	printk(KERN_NOTICE "NetSc520 flash device: 0x%Lx at 0x%Lx\n",
			(unsigned long long)netsc520_map.size,
			(unsigned long long)netsc520_map.phys);
	netsc520_map.virt = ioremap_nocache(netsc520_map.phys, netsc520_map.size);

	if (!netsc520_map.virt) {
		printk("Failed to ioremap_nocache\n");
		return -EIO;
	}

	simple_map_init(&netsc520_map);

	mymtd = do_map_probe("cfi_probe", &netsc520_map);
	if(!mymtd)
		mymtd = do_map_probe("map_ram", &netsc520_map);
	if(!mymtd)
		mymtd = do_map_probe("map_rom", &netsc520_map);

	if (!mymtd) {
		iounmap(netsc520_map.virt);
		return -ENXIO;
	}

	mymtd->owner = THIS_MODULE;
	add_mtd_partitions( mymtd, partition_info, NUM_PARTITIONS );
	return 0;
}

static void __exit cleanup_netsc520(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
	if (netsc520_map.virt) {
		iounmap(netsc520_map.virt);
		netsc520_map.virt = NULL;
	}
}

module_init(init_netsc520);
module_exit(cleanup_netsc520);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mark Langsdorf <mark.langsdorf@amd.com>");
MODULE_DESCRIPTION("MTD map driver for AMD NetSc520 Demonstration Board");
