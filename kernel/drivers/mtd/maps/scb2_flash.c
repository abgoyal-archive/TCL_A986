

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>

#define MODNAME		"scb2_flash"
#define SCB2_ADDR	0xfff00000
#define SCB2_WINDOW	0x00100000


static void __iomem *scb2_ioaddr;
static struct mtd_info *scb2_mtd;
static struct map_info scb2_map = {
	.name =      "SCB2 BIOS Flash",
	.size =      0,
	.bankwidth =  1,
};
static int region_fail;

static int __devinit
scb2_fixup_mtd(struct mtd_info *mtd)
{
	int i;
	int done = 0;
	struct map_info *map = mtd->priv;
	struct cfi_private *cfi = map->fldrv_priv;

	/* barf if this doesn't look right */
	if (cfi->cfiq->InterfaceDesc != CFI_INTERFACE_X16_ASYNC) {
		printk(KERN_ERR MODNAME ": unsupported InterfaceDesc: %#x\n",
		    cfi->cfiq->InterfaceDesc);
		return -1;
	}

	/* I wasn't here. I didn't see. dwmw2. */

	/* the chip is sometimes bigger than the map - what a waste */
	mtd->size = map->size;

	/*
	 * We only REALLY get half the chip, due to the way it is
	 * wired up - D8-D15 are tossed away.  We read linear bytes,
	 * but in reality we are getting 1/2 of each 16-bit read,
	 * which LOOKS linear to us.  Because CFI code accounts for
	 * things like lock/unlock/erase by eraseregions, we need to
	 * fudge them to reflect this.  Erases go like this:
	 *   * send an erase to an address
	 *   * the chip samples the address and erases the block
	 *   * add the block erasesize to the address and repeat
	 *   -- the problem is that addresses are 16-bit addressable
	 *   -- we end up erasing every-other block
	 */
	mtd->erasesize /= 2;
	for (i = 0; i < mtd->numeraseregions; i++) {
		struct mtd_erase_region_info *region = &mtd->eraseregions[i];
		region->erasesize /= 2;
	}

	/*
	 * If the chip is bigger than the map, it is wired with the high
	 * address lines pulled up.  This makes us access the top portion of
	 * the chip, so all our erase-region info is wrong.  Start cutting from
	 * the bottom.
	 */
	for (i = 0; !done && i < mtd->numeraseregions; i++) {
		struct mtd_erase_region_info *region = &mtd->eraseregions[i];

		if (region->numblocks * region->erasesize > mtd->size) {
			region->numblocks = ((unsigned long)mtd->size /
						region->erasesize);
			done = 1;
		} else {
			region->numblocks = 0;
		}
		region->offset = 0;
	}

	return 0;
}

/* CSB5's 'Function Control Register' has bits for decoding @ >= 0xffc00000 */
#define CSB5_FCR	0x41
#define CSB5_FCR_DECODE_ALL 0x0e
static int __devinit
scb2_flash_probe(struct pci_dev *dev, const struct pci_device_id *ent)
{
	u8 reg;

	/* enable decoding of the flash region in the south bridge */
	pci_read_config_byte(dev, CSB5_FCR, &reg);
	pci_write_config_byte(dev, CSB5_FCR, reg | CSB5_FCR_DECODE_ALL);

	if (!request_mem_region(SCB2_ADDR, SCB2_WINDOW, scb2_map.name)) {
		/*
		 * The BIOS seems to mark the flash region as 'reserved'
		 * in the e820 map.  Warn and go about our business.
		 */
		printk(KERN_WARNING MODNAME
		    ": warning - can't reserve rom window, continuing\n");
		region_fail = 1;
	}

	/* remap the IO window (w/o caching) */
	scb2_ioaddr = ioremap_nocache(SCB2_ADDR, SCB2_WINDOW);
	if (!scb2_ioaddr) {
		printk(KERN_ERR MODNAME ": Failed to ioremap window!\n");
		if (!region_fail)
			release_mem_region(SCB2_ADDR, SCB2_WINDOW);
		return -ENOMEM;
	}

	scb2_map.phys = SCB2_ADDR;
	scb2_map.virt = scb2_ioaddr;
	scb2_map.size = SCB2_WINDOW;

	simple_map_init(&scb2_map);

	/* try to find a chip */
	scb2_mtd = do_map_probe("cfi_probe", &scb2_map);

	if (!scb2_mtd) {
		printk(KERN_ERR MODNAME ": flash probe failed!\n");
		iounmap(scb2_ioaddr);
		if (!region_fail)
			release_mem_region(SCB2_ADDR, SCB2_WINDOW);
		return -ENODEV;
	}

	scb2_mtd->owner = THIS_MODULE;
	if (scb2_fixup_mtd(scb2_mtd) < 0) {
		del_mtd_device(scb2_mtd);
		map_destroy(scb2_mtd);
		iounmap(scb2_ioaddr);
		if (!region_fail)
			release_mem_region(SCB2_ADDR, SCB2_WINDOW);
		return -ENODEV;
	}

	printk(KERN_NOTICE MODNAME ": chip size 0x%llx at offset 0x%llx\n",
	       (unsigned long long)scb2_mtd->size,
	       (unsigned long long)(SCB2_WINDOW - scb2_mtd->size));

	add_mtd_device(scb2_mtd);

	return 0;
}

static void __devexit
scb2_flash_remove(struct pci_dev *dev)
{
	if (!scb2_mtd)
		return;

	/* disable flash writes */
	if (scb2_mtd->lock)
		scb2_mtd->lock(scb2_mtd, 0, scb2_mtd->size);

	del_mtd_device(scb2_mtd);
	map_destroy(scb2_mtd);

	iounmap(scb2_ioaddr);
	scb2_ioaddr = NULL;

	if (!region_fail)
		release_mem_region(SCB2_ADDR, SCB2_WINDOW);
	pci_set_drvdata(dev, NULL);
}

static struct pci_device_id scb2_flash_pci_ids[] = {
	{
	  .vendor = PCI_VENDOR_ID_SERVERWORKS,
	  .device = PCI_DEVICE_ID_SERVERWORKS_CSB5,
	  .subvendor = PCI_ANY_ID,
	  .subdevice = PCI_ANY_ID
	},
	{ 0, }
};

static struct pci_driver scb2_flash_driver = {
	.name =     "Intel SCB2 BIOS Flash",
	.id_table = scb2_flash_pci_ids,
	.probe =    scb2_flash_probe,
	.remove =   __devexit_p(scb2_flash_remove),
};

static int __init
scb2_flash_init(void)
{
	return pci_register_driver(&scb2_flash_driver);
}

static void __exit
scb2_flash_exit(void)
{
	pci_unregister_driver(&scb2_flash_driver);
}

module_init(scb2_flash_init);
module_exit(scb2_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Hockin <thockin@sun.com>");
MODULE_DESCRIPTION("MTD map driver for Intel SCB2 BIOS Flash");
MODULE_DEVICE_TABLE(pci, scb2_flash_pci_ids);
