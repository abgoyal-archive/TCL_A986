

#define DRIVER_NAME "orinoco_pci"
#define PFX DRIVER_NAME ": "

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "orinoco.h"
#include "orinoco_pci.h"

/* Offset of the COR register of the PCI card */
#define HERMES_PCI_COR		(0x26)

/* Bitmask to reset the card */
#define HERMES_PCI_COR_MASK	(0x0080)

#define HERMES_PCI_COR_ONT	(250)		/* ms */
#define HERMES_PCI_COR_OFFT	(500)		/* ms */
#define HERMES_PCI_COR_BUSYT	(500)		/* ms */

static int orinoco_pci_cor_reset(struct orinoco_private *priv)
{
	hermes_t *hw = &priv->hw;
	unsigned long timeout;
	u16 reg;

	/* Assert the reset until the card notices */
	hermes_write_regn(hw, PCI_COR, HERMES_PCI_COR_MASK);
	mdelay(HERMES_PCI_COR_ONT);

	/* Give time for the card to recover from this hard effort */
	hermes_write_regn(hw, PCI_COR, 0x0000);
	mdelay(HERMES_PCI_COR_OFFT);

	/* The card is ready when it's no longer busy */
	timeout = jiffies + (HERMES_PCI_COR_BUSYT * HZ / 1000);
	reg = hermes_read_regn(hw, CMD);
	while (time_before(jiffies, timeout) && (reg & HERMES_CMD_BUSY)) {
		mdelay(1);
		reg = hermes_read_regn(hw, CMD);
	}

	/* Still busy? */
	if (reg & HERMES_CMD_BUSY) {
		printk(KERN_ERR PFX "Busy timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int orinoco_pci_init_one(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	int err;
	struct orinoco_private *priv;
	struct orinoco_pci_card *card;
	void __iomem *hermes_io;

	err = pci_enable_device(pdev);
	if (err) {
		printk(KERN_ERR PFX "Cannot enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, DRIVER_NAME);
	if (err) {
		printk(KERN_ERR PFX "Cannot obtain PCI resources\n");
		goto fail_resources;
	}

	hermes_io = pci_iomap(pdev, 0, 0);
	if (!hermes_io) {
		printk(KERN_ERR PFX "Cannot remap chipset registers\n");
		err = -EIO;
		goto fail_map_hermes;
	}

	/* Allocate network device */
	priv = alloc_orinocodev(sizeof(*card), &pdev->dev,
				orinoco_pci_cor_reset, NULL);
	if (!priv) {
		printk(KERN_ERR PFX "Cannot allocate network device\n");
		err = -ENOMEM;
		goto fail_alloc;
	}

	card = priv->card;

	hermes_struct_init(&priv->hw, hermes_io, HERMES_32BIT_REGSPACING);

	err = request_irq(pdev->irq, orinoco_interrupt, IRQF_SHARED,
			  DRIVER_NAME, priv);
	if (err) {
		printk(KERN_ERR PFX "Cannot allocate IRQ %d\n", pdev->irq);
		err = -EBUSY;
		goto fail_irq;
	}

	err = orinoco_pci_cor_reset(priv);
	if (err) {
		printk(KERN_ERR PFX "Initial reset failed\n");
		goto fail;
	}

	err = orinoco_init(priv);
	if (err) {
		printk(KERN_ERR PFX "orinoco_init() failed\n");
		goto fail;
	}

	err = orinoco_if_add(priv, 0, 0, NULL);
	if (err) {
		printk(KERN_ERR PFX "orinoco_if_add() failed\n");
		goto fail;
	}

	pci_set_drvdata(pdev, priv);

	return 0;

 fail:
	free_irq(pdev->irq, priv);

 fail_irq:
	pci_set_drvdata(pdev, NULL);
	free_orinocodev(priv);

 fail_alloc:
	pci_iounmap(pdev, hermes_io);

 fail_map_hermes:
	pci_release_regions(pdev);

 fail_resources:
	pci_disable_device(pdev);

	return err;
}

static void __devexit orinoco_pci_remove_one(struct pci_dev *pdev)
{
	struct orinoco_private *priv = pci_get_drvdata(pdev);

	orinoco_if_del(priv);
	free_irq(pdev->irq, priv);
	pci_set_drvdata(pdev, NULL);
	free_orinocodev(priv);
	pci_iounmap(pdev, priv->hw.iobase);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static DEFINE_PCI_DEVICE_TABLE(orinoco_pci_id_table) = {
	/* Intersil Prism 3 */
	{0x1260, 0x3872, PCI_ANY_ID, PCI_ANY_ID,},
	/* Intersil Prism 2.5 */
	{0x1260, 0x3873, PCI_ANY_ID, PCI_ANY_ID,},
	/* Samsung MagicLAN SWL-2210P */
	{0x167d, 0xa000, PCI_ANY_ID, PCI_ANY_ID,},
	{0,},
};

MODULE_DEVICE_TABLE(pci, orinoco_pci_id_table);

static struct pci_driver orinoco_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= orinoco_pci_id_table,
	.probe		= orinoco_pci_init_one,
	.remove		= __devexit_p(orinoco_pci_remove_one),
	.suspend	= orinoco_pci_suspend,
	.resume		= orinoco_pci_resume,
};

static char version[] __initdata = DRIVER_NAME " " DRIVER_VERSION
	" (Pavel Roskin <proski@gnu.org>,"
	" David Gibson <hermes@gibson.dropbear.id.au> &"
	" Jean Tourrilhes <jt@hpl.hp.com>)";
MODULE_AUTHOR("Pavel Roskin <proski@gnu.org> &"
	      " David Gibson <hermes@gibson.dropbear.id.au>");
MODULE_DESCRIPTION("Driver for wireless LAN cards using direct PCI interface");
MODULE_LICENSE("Dual MPL/GPL");

static int __init orinoco_pci_init(void)
{
	printk(KERN_DEBUG "%s\n", version);
	return pci_register_driver(&orinoco_pci_driver);
}

static void __exit orinoco_pci_exit(void)
{
	pci_unregister_driver(&orinoco_pci_driver);
}

module_init(orinoco_pci_init);
module_exit(orinoco_pci_exit);

