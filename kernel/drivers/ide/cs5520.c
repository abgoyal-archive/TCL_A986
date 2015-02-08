
 
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/ide.h>
#include <linux/dma-mapping.h>

#define DRV_NAME "cs5520"

struct pio_clocks
{
	int address;
	int assert;
	int recovery;
};

static struct pio_clocks cs5520_pio_clocks[]={
	{3, 6, 11},
	{2, 5, 6},
	{1, 4, 3},
	{1, 3, 2},
	{1, 2, 1}
};

static void cs5520_set_pio_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	struct pci_dev *pdev = to_pci_dev(hwif->dev);
	int controller = drive->dn > 1 ? 1 : 0;
	const u8 pio = drive->pio_mode - XFER_PIO_0;

	/* 8bit CAT/CRT - 8bit command timing for channel */
	pci_write_config_byte(pdev, 0x62 + controller, 
		(cs5520_pio_clocks[pio].recovery << 4) |
		(cs5520_pio_clocks[pio].assert));

	/* 0x64 - 16bit Primary, 0x68 - 16bit Secondary */

	/* FIXME: should these use address ? */
	/* Data read timing */
	pci_write_config_byte(pdev, 0x64 + 4*controller + (drive->dn&1),
		(cs5520_pio_clocks[pio].recovery << 4) |
		(cs5520_pio_clocks[pio].assert));
	/* Write command timing */
	pci_write_config_byte(pdev, 0x66 + 4*controller + (drive->dn&1),
		(cs5520_pio_clocks[pio].recovery << 4) |
		(cs5520_pio_clocks[pio].assert));
}

static void cs5520_set_dma_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	printk(KERN_ERR "cs55x0: bad ide timing.\n");

	drive->pio_mode = XFER_PIO_0 + 0;
	cs5520_set_pio_mode(hwif, drive);
}

static const struct ide_port_ops cs5520_port_ops = {
	.set_pio_mode		= cs5520_set_pio_mode,
	.set_dma_mode		= cs5520_set_dma_mode,
};

static const struct ide_port_info cyrix_chipset __devinitdata = {
	.name		= DRV_NAME,
	.enablebits	= { { 0x60, 0x01, 0x01 }, { 0x60, 0x02, 0x02 } },
	.port_ops	= &cs5520_port_ops,
	.host_flags	= IDE_HFLAG_ISA_PORTS | IDE_HFLAG_CS5520,
	.pio_mask	= ATA_PIO4,
};

 
static int __devinit cs5520_init_one(struct pci_dev *dev, const struct pci_device_id *id)
{
	const struct ide_port_info *d = &cyrix_chipset;
	struct ide_hw hw[2], *hws[] = { NULL, NULL };

	ide_setup_pci_noise(dev, d);

	/* We must not grab the entire device, it has 'ISA' space in its
	 * BARS too and we will freak out other bits of the kernel
	 */
	if (pci_enable_device_io(dev)) {
		printk(KERN_WARNING "%s: Unable to enable 55x0.\n", d->name);
		return -ENODEV;
	}
	pci_set_master(dev);
	if (pci_set_dma_mask(dev, DMA_BIT_MASK(32))) {
		printk(KERN_WARNING "%s: No suitable DMA available.\n",
			d->name);
		return -ENODEV;
	}

	/*
	 *	Now the chipset is configured we can let the core
	 *	do all the device setup for us
	 */

	ide_pci_setup_ports(dev, d, &hw[0], &hws[0]);
	hw[0].irq = 14;
	hw[1].irq = 15;

	return ide_host_add(d, hws, 2, NULL);
}

static const struct pci_device_id cs5520_pci_tbl[] = {
	{ PCI_VDEVICE(CYRIX, PCI_DEVICE_ID_CYRIX_5510), 0 },
	{ PCI_VDEVICE(CYRIX, PCI_DEVICE_ID_CYRIX_5520), 1 },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, cs5520_pci_tbl);

static struct pci_driver cs5520_pci_driver = {
	.name		= "Cyrix_IDE",
	.id_table	= cs5520_pci_tbl,
	.probe		= cs5520_init_one,
	.suspend	= ide_pci_suspend,
	.resume		= ide_pci_resume,
};

static int __init cs5520_ide_init(void)
{
	return ide_pci_register_driver(&cs5520_pci_driver);
}

module_init(cs5520_ide_init);

MODULE_AUTHOR("Alan Cox");
MODULE_DESCRIPTION("PCI driver module for Cyrix 5510/5520 IDE");
MODULE_LICENSE("GPL");
