

#include <linux/init.h>
#include <linux/pnp.h>
#include <linux/ide.h>

#define DRV_NAME "ide-pnp"

/* Add your devices here :)) */
static struct pnp_device_id idepnp_devices[] = {
	/* Generic ESDI/IDE/ATA compatible hard disk controller */
	{.id = "PNP0600", .driver_data = 0},
	{.id = ""}
};

static const struct ide_port_info ide_pnp_port_info = {
	.host_flags		= IDE_HFLAG_NO_DMA,
	.chipset		= ide_generic,
};

static int idepnp_probe(struct pnp_dev *dev, const struct pnp_device_id *dev_id)
{
	struct ide_host *host;
	unsigned long base, ctl;
	int rc;
	struct ide_hw hw, *hws[] = { &hw };

	printk(KERN_INFO DRV_NAME ": generic PnP IDE interface\n");

	if (!(pnp_port_valid(dev, 0) && pnp_port_valid(dev, 1) && pnp_irq_valid(dev, 0)))
		return -1;

	base = pnp_port_start(dev, 0);
	ctl = pnp_port_start(dev, 1);

	if (!request_region(base, 8, DRV_NAME)) {
		printk(KERN_ERR "%s: I/O resource 0x%lX-0x%lX not free.\n",
				DRV_NAME, base, base + 7);
		return -EBUSY;
	}

	if (!request_region(ctl, 1, DRV_NAME)) {
		printk(KERN_ERR "%s: I/O resource 0x%lX not free.\n",
				DRV_NAME, ctl);
		release_region(base, 8);
		return -EBUSY;
	}

	memset(&hw, 0, sizeof(hw));
	ide_std_init_ports(&hw, base, ctl);
	hw.irq = pnp_irq(dev, 0);

	rc = ide_host_add(&ide_pnp_port_info, hws, 1, &host);
	if (rc)
		goto out;

	pnp_set_drvdata(dev, host);

	return 0;
out:
	release_region(ctl, 1);
	release_region(base, 8);

	return rc;
}

static void idepnp_remove(struct pnp_dev *dev)
{
	struct ide_host *host = pnp_get_drvdata(dev);

	ide_host_remove(host);

	release_region(pnp_port_start(dev, 1), 1);
	release_region(pnp_port_start(dev, 0), 8);
}

static struct pnp_driver idepnp_driver = {
	.name		= "ide",
	.id_table	= idepnp_devices,
	.probe		= idepnp_probe,
	.remove		= idepnp_remove,
};

static int __init pnpide_init(void)
{
	return pnp_register_driver(&idepnp_driver);
}

static void __exit pnpide_exit(void)
{
	pnp_unregister_driver(&idepnp_driver);
}

module_init(pnpide_init);
module_exit(pnpide_exit);

MODULE_LICENSE("GPL");
