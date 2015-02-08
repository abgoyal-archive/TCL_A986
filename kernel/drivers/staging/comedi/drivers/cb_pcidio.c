

/*------------------------------ HEADER FILES ---------------------------------*/
#include "../comedidev.h"
#include "comedi_pci.h"
#include "8255.h"

/*-------------------------- MACROS and DATATYPES -----------------------------*/
#define PCI_VENDOR_ID_CB	0x1307

struct pcidio_board {
	const char *name;	/*  name of the board */
	int dev_id;
	int n_8255;		/*  number of 8255 chips on board */

	/*  indices of base address regions */
	int pcicontroler_badrindex;
	int dioregs_badrindex;
};

static const struct pcidio_board pcidio_boards[] = {
	{
	 .name = "pci-dio24",
	 .dev_id = 0x0028,
	 .n_8255 = 1,
	 .pcicontroler_badrindex = 1,
	 .dioregs_badrindex = 2,
	 },
	{
	 .name = "pci-dio24h",
	 .dev_id = 0x0014,
	 .n_8255 = 1,
	 .pcicontroler_badrindex = 1,
	 .dioregs_badrindex = 2,
	 },
	{
	 .name = "pci-dio48h",
	 .dev_id = 0x000b,
	 .n_8255 = 2,
	 .pcicontroler_badrindex = 0,
	 .dioregs_badrindex = 1,
	 },
};

static DEFINE_PCI_DEVICE_TABLE(pcidio_pci_table) = {
	{
	PCI_VENDOR_ID_CB, 0x0028, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	PCI_VENDOR_ID_CB, 0x0014, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	PCI_VENDOR_ID_CB, 0x000b, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	0}
};

MODULE_DEVICE_TABLE(pci, pcidio_pci_table);

#define thisboard ((const struct pcidio_board *)dev->board_ptr)

struct pcidio_private {
	int data;		/*  currently unused */

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	/* used for DO readback, currently unused */
	unsigned int do_readback[4];	/* up to 4 unsigned int suffice to hold 96 bits for PCI-DIO96 */

	unsigned long dio_reg_base;	/*  address of port A of the first 8255 chip on board */
};

#define devpriv ((struct pcidio_private *)dev->private)

static int pcidio_attach(struct comedi_device *dev,
			 struct comedi_devconfig *it);
static int pcidio_detach(struct comedi_device *dev);
static struct comedi_driver driver_cb_pcidio = {
	.driver_name = "cb_pcidio",
	.module = THIS_MODULE,
	.attach = pcidio_attach,
	.detach = pcidio_detach,


	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in pcidio_boards[] above.  Note that the element 'name'
	 * was first in the structure -- Comedi uses this fact to
	 * extract the name of the board without knowing any details
	 * about the structure except for its length.
	 * When a device is attached (by comedi_config), the name
	 * of the device is given to Comedi, and Comedi tries to
	 * match it by going through the list of board names.  If
	 * there is a match, the address of the pointer is put
	 * into dev->board_ptr and driver->attach() is called.
	 *
	 * Note that these are not necessary if you can determine
	 * the type of board in software.  ISA PnP, PCI, and PCMCIA
	 * devices are such boards.
	 */


};

/*------------------------------- FUNCTIONS -----------------------------------*/

static int pcidio_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	struct pci_dev *pcidev = NULL;
	int index;
	int i;

	printk("comedi%d: cb_pcidio: \n", dev->minor);

	if (alloc_private(dev, sizeof(struct pcidio_private)) < 0)
		return -ENOMEM;

	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
	     pcidev != NULL;
	     pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {
		/*  is it not a computer boards card? */
		if (pcidev->vendor != PCI_VENDOR_ID_CB)
			continue;
		/*  loop through cards supported by this driver */
		for (index = 0; index < ARRAY_SIZE(pcidio_boards); index++) {
			if (pcidio_boards[index].dev_id != pcidev->device)
				continue;

			/*  was a particular bus/slot requested? */
			if (it->options[0] || it->options[1]) {
				/*  are we on the wrong bus/slot? */
				if (pcidev->bus->number != it->options[0] ||
				    PCI_SLOT(pcidev->devfn) != it->options[1]) {
					continue;
				}
			}
			dev->board_ptr = pcidio_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found on "
	       "requested position\n");
	return -EIO;

found:

	dev->board_name = thisboard->name;

	devpriv->pci_dev = pcidev;
	printk("Found %s on bus %i, slot %i\n", thisboard->name,
	       devpriv->pci_dev->bus->number,
	       PCI_SLOT(devpriv->pci_dev->devfn));
	if (comedi_pci_enable(pcidev, thisboard->name)) {
		printk
		    ("cb_pcidio: failed to enable PCI device and request regions\n");
		return -EIO;
	}
	devpriv->dio_reg_base
	    =
	    pci_resource_start(devpriv->pci_dev,
			       pcidio_boards[index].dioregs_badrindex);

	if (alloc_subdevices(dev, thisboard->n_8255) < 0)
		return -ENOMEM;

	for (i = 0; i < thisboard->n_8255; i++) {
		subdev_8255_init(dev, dev->subdevices + i,
				 NULL, devpriv->dio_reg_base + i * 4);
		printk(" subdev %d: base = 0x%lx\n", i,
		       devpriv->dio_reg_base + i * 4);
	}

	printk("attached\n");
	return 1;
}

static int pcidio_detach(struct comedi_device *dev)
{
	printk("comedi%d: cb_pcidio: remove\n", dev->minor);
	if (devpriv) {
		if (devpriv->pci_dev) {
			if (devpriv->dio_reg_base)
				comedi_pci_disable(devpriv->pci_dev);
			pci_dev_put(devpriv->pci_dev);
		}
	}
	if (dev->subdevices) {
		int i;
		for (i = 0; i < thisboard->n_8255; i++)
			subdev_8255_cleanup(dev, dev->subdevices + i);
	}
	return 0;
}

COMEDI_PCI_INITCLEANUP(driver_cb_pcidio, pcidio_pci_table);
