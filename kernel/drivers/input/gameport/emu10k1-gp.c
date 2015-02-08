



#include <asm/io.h>

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/gameport.h>
#include <linux/slab.h>
#include <linux/pci.h>

MODULE_AUTHOR("Vojtech Pavlik <vojtech@ucw.cz>");
MODULE_DESCRIPTION("EMU10k1 gameport driver");
MODULE_LICENSE("GPL");

struct emu {
	struct pci_dev *dev;
	struct gameport *gameport;
	int io;
	int size;
};

static const struct pci_device_id emu_tbl[] = {

	{ 0x1102, 0x7002, PCI_ANY_ID, PCI_ANY_ID }, /* SB Live gameport */
	{ 0x1102, 0x7003, PCI_ANY_ID, PCI_ANY_ID }, /* Audigy gameport */
	{ 0x1102, 0x7004, PCI_ANY_ID, PCI_ANY_ID }, /* Dell SB Live */
	{ 0x1102, 0x7005, PCI_ANY_ID, PCI_ANY_ID }, /* Audigy LS gameport */
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, emu_tbl);

static int __devinit emu_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ioport, iolen;
	struct emu *emu;
	struct gameport *port;

	if (pci_enable_device(pdev))
		return -EBUSY;

	ioport = pci_resource_start(pdev, 0);
	iolen = pci_resource_len(pdev, 0);

	if (!request_region(ioport, iolen, "emu10k1-gp"))
		return -EBUSY;

	emu = kzalloc(sizeof(struct emu), GFP_KERNEL);
	port = gameport_allocate_port();
	if (!emu || !port) {
		printk(KERN_ERR "emu10k1-gp: Memory allocation failed\n");
		release_region(ioport, iolen);
		kfree(emu);
		gameport_free_port(port);
		return -ENOMEM;
	}

	emu->io = ioport;
	emu->size = iolen;
	emu->dev = pdev;
	emu->gameport = port;

	gameport_set_name(port, "EMU10K1");
	gameport_set_phys(port, "pci%s/gameport0", pci_name(pdev));
	port->dev.parent = &pdev->dev;
	port->io = ioport;

	pci_set_drvdata(pdev, emu);

	gameport_register_port(port);

	return 0;
}

static void __devexit emu_remove(struct pci_dev *pdev)
{
	struct emu *emu = pci_get_drvdata(pdev);

	gameport_unregister_port(emu->gameport);
	release_region(emu->io, emu->size);
	kfree(emu);
}

static struct pci_driver emu_driver = {
        .name =         "Emu10k1_gameport",
        .id_table =     emu_tbl,
        .probe =        emu_probe,
        .remove =       __devexit_p(emu_remove),
};

static int __init emu_init(void)
{
	return pci_register_driver(&emu_driver);
}

static void __exit emu_exit(void)
{
	pci_unregister_driver(&emu_driver);
}

module_init(emu_init);
module_exit(emu_exit);
