

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/autcpu12.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


static struct mtd_info *sram_mtd;

struct map_info autcpu12_sram_map = {
	.name = "SRAM",
	.size = 32768,
	.bankwidth = 4,
	.phys = 0x12000000,
};

static int __init init_autcpu12_sram (void)
{
	int err, save0, save1;

	autcpu12_sram_map.virt = ioremap(0x12000000, SZ_128K);
	if (!autcpu12_sram_map.virt) {
		printk("Failed to ioremap autcpu12 NV-RAM space\n");
		err = -EIO;
		goto out;
	}
	simple_map_init(&autcpu_sram_map);

	/*
	 * Check for 32K/128K
	 * read ofs 0
	 * read ofs 0x10000
	 * Write complement to ofs 0x100000
	 * Read	and check result on ofs 0x0
	 * Restore contents
	 */
	save0 = map_read32(&autcpu12_sram_map,0);
	save1 = map_read32(&autcpu12_sram_map,0x10000);
	map_write32(&autcpu12_sram_map,~save0,0x10000);
	/* if we find this pattern on 0x0, we have 32K size
	 * restore contents and exit
	 */
	if ( map_read32(&autcpu12_sram_map,0) != save0) {
		map_write32(&autcpu12_sram_map,save0,0x0);
		goto map;
	}
	/* We have a 128K found, restore 0x10000 and set size
	 * to 128K
	 */
	map_write32(&autcpu12_sram_map,save1,0x10000);
	autcpu12_sram_map.size = SZ_128K;

map:
	sram_mtd = do_map_probe("map_ram", &autcpu12_sram_map);
	if (!sram_mtd) {
		printk("NV-RAM probe failed\n");
		err = -ENXIO;
		goto out_ioremap;
	}

	sram_mtd->owner = THIS_MODULE;
	sram_mtd->erasesize = 16;

	if (add_mtd_device(sram_mtd)) {
		printk("NV-RAM device addition failed\n");
		err = -ENOMEM;
		goto out_probe;
	}

	printk("NV-RAM device size %ldKiB registered on AUTCPU12\n",autcpu12_sram_map.size/SZ_1K);

	return 0;

out_probe:
	map_destroy(sram_mtd);
	sram_mtd = 0;

out_ioremap:
	iounmap((void *)autcpu12_sram_map.virt);
out:
	return err;
}

static void __exit cleanup_autcpu12_maps(void)
{
	if (sram_mtd) {
		del_mtd_device(sram_mtd);
		map_destroy(sram_mtd);
		iounmap((void *)autcpu12_sram_map.virt);
	}
}

module_init(init_autcpu12_sram);
module_exit(cleanup_autcpu12_maps);

MODULE_AUTHOR("Thomas Gleixner");
MODULE_DESCRIPTION("autcpu12 NV-RAM map driver");
MODULE_LICENSE("GPL");
