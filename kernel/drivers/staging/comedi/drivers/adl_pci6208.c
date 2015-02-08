
#include "../comedidev.h"
#include "comedi_pci.h"

#define PCI6208_DRIVER_NAME	"adl_pci6208"

/* Board descriptions */
struct pci6208_board {
	const char *name;
	unsigned short dev_id;	/* `lspci` will show you this */
	int ao_chans;
	/* int ao_bits; */
};

static const struct pci6208_board pci6208_boards[] = {
	/*{
	   .name = "pci6208v",
	   .dev_id = 0x6208,      // not sure
	   .ao_chans = 8
	   // , .ao_bits = 16
	   },
	   {
	   .name = "pci6216v",
	   .dev_id = 0x6208,      // not sure
	   .ao_chans = 16
	   // , .ao_bits = 16
	   }, */
	{
	 .name = "pci6208a",
	 .dev_id = 0x6208,
	 .ao_chans = 8
	 /* ,    .ao_bits = 16 */
	 }
};

static DEFINE_PCI_DEVICE_TABLE(pci6208_pci_table) = {
	/* { PCI_VENDOR_ID_ADLINK, 0x6208, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 }, */
	/* { PCI_VENDOR_ID_ADLINK, 0x6208, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 }, */
	{
	PCI_VENDOR_ID_ADLINK, 0x6208, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	0}
};

MODULE_DEVICE_TABLE(pci, pci6208_pci_table);

/* Will be initialized in pci6208_find device(). */
#define thisboard ((const struct pci6208_board *)dev->board_ptr)

struct pci6208_private {
	int data;
	struct pci_dev *pci_dev;	/* for a PCI device */
	unsigned int ao_readback[2];	/* Used for AO readback */
};

#define devpriv ((struct pci6208_private *)dev->private)

static int pci6208_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it);
static int pci6208_detach(struct comedi_device *dev);

static struct comedi_driver driver_pci6208 = {
	.driver_name = PCI6208_DRIVER_NAME,
	.module = THIS_MODULE,
	.attach = pci6208_attach,
	.detach = pci6208_detach,
};

COMEDI_PCI_INITCLEANUP(driver_pci6208, pci6208_pci_table);

static int pci6208_find_device(struct comedi_device *dev, int bus, int slot);
static int
pci6208_pci_setup(struct pci_dev *pci_dev, unsigned long *io_base_ptr,
		  int dev_minor);

/*read/write functions*/
static int pci6208_ao_winsn(struct comedi_device *dev,
			    struct comedi_subdevice *s,
			    struct comedi_insn *insn, unsigned int *data);
static int pci6208_ao_rinsn(struct comedi_device *dev,
			    struct comedi_subdevice *s,
			    struct comedi_insn *insn, unsigned int *data);
/* struct comedi_insn *insn,unsigned int *data); */
/* struct comedi_insn *insn,unsigned int *data); */

static int pci6208_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;
	int retval;
	unsigned long io_base;

	printk(KERN_INFO "comedi%d: pci6208: ", dev->minor);

	retval = alloc_private(dev, sizeof(struct pci6208_private));
	if (retval < 0)
		return retval;

	retval = pci6208_find_device(dev, it->options[0], it->options[1]);
	if (retval < 0)
		return retval;

	retval = pci6208_pci_setup(devpriv->pci_dev, &io_base, dev->minor);
	if (retval < 0)
		return retval;

	dev->iobase = io_base;
	dev->board_name = thisboard->name;

	if (alloc_subdevices(dev, 2) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog output subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;	/* anything else to add here?? */
	s->n_chan = thisboard->ao_chans;
	s->maxdata = 0xffff;	/* 16-bit DAC */
	s->range_table = &range_bipolar10;	/* this needs to be checked. */
	s->insn_write = pci6208_ao_winsn;
	s->insn_read = pci6208_ao_rinsn;

	/* s=dev->subdevices+1; */
	/* digital i/o subdevice */
	/* s->type=COMEDI_SUBD_DIO; */
	/* s->subdev_flags=SDF_READABLE|SDF_WRITABLE; */
	/* s->n_chan=16; */
	/* s->maxdata=1; */
	/* s->range_table=&range_digital; */
	/* s->insn_bits = pci6208_dio_insn_bits; */
	/* s->insn_config = pci6208_dio_insn_config; */

	printk(KERN_INFO "attached\n");

	return 1;
}

static int pci6208_detach(struct comedi_device *dev)
{
	printk(KERN_INFO "comedi%d: pci6208: remove\n", dev->minor);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase)
			comedi_pci_disable(devpriv->pci_dev);
		pci_dev_put(devpriv->pci_dev);
	}

	return 0;
}

static int pci6208_ao_winsn(struct comedi_device *dev,
			    struct comedi_subdevice *s,
			    struct comedi_insn *insn, unsigned int *data)
{
	int i = 0, Data_Read;
	unsigned short chan = CR_CHAN(insn->chanspec);
	unsigned long invert = 1 << (16 - 1);
	unsigned long out_value;
	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for (i = 0; i < insn->n; i++) {
		out_value = data[i] ^ invert;
		/* a typical programming sequence */
		do {
			Data_Read = (inw(dev->iobase) & 1);
		} while (Data_Read);
		outw(out_value, dev->iobase + (0x02 * chan));
		devpriv->ao_readback[chan] = out_value;
	}

	/* return the number of samples read/written */
	return i;
}

static int pci6208_ao_rinsn(struct comedi_device *dev,
			    struct comedi_subdevice *s,
			    struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

/* struct comedi_insn *insn,unsigned int *data) */
/* { */
/* if(insn->n!=2)return -EINVAL; */

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
/* if(data[0]){ */
/* s->state &= ~data[0]; */
/* s->state |= data[0]&data[1]; */
		/* Write out the new digital output lines */
		/* outw(s->state,dev->iobase + SKEL_DIO); */
/* } */

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	/* data[1]=inw(dev->iobase + SKEL_DIO); */
	/* or we could just return the software copy of the output values if
	 * it was a purely digital output subdevice */
	/* data[1]=s->state; */

/* return 2; */
/* } */

/* struct comedi_insn *insn,unsigned int *data) */
/* { */
/* int chan=CR_CHAN(insn->chanspec); */

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */

/* if(data[0]==COMEDI_OUTPUT){ */
/* s->io_bits |= 1<<chan; */
/* }else{ */
/* s->io_bits &= ~(1<<chan); */
/* } */
	/* outw(s->io_bits,dev->iobase + SKEL_DIO_CONFIG); */

/* return 1; */
/* } */

static int pci6208_find_device(struct comedi_device *dev, int bus, int slot)
{
	struct pci_dev *pci_dev;
	int i;

	for (pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
	     pci_dev != NULL;
	     pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_dev)) {
		if (pci_dev->vendor == PCI_VENDOR_ID_ADLINK) {
			for (i = 0; i < ARRAY_SIZE(pci6208_boards); i++) {
				if (pci6208_boards[i].dev_id ==
					pci_dev->device) {
					/*
					 * was a particular bus/slot requested?
					*/
					if ((bus != 0) || (slot != 0)) {
						/*
						 * are we on the
						 * wrong bus/slot?
						*/
						if (pci_dev->bus->number
						    != bus ||
						    PCI_SLOT(pci_dev->devfn)
						    != slot) {
							continue;
						}
					}
					dev->board_ptr = pci6208_boards + i;
					goto found;
				}
			}
		}
	}

	printk(KERN_ERR "comedi%d: no supported board found! "
			"(req. bus/slot : %d/%d)\n",
			dev->minor, bus, slot);
	return -EIO;

found:
	printk("comedi%d: found %s (b:s:f=%d:%d:%d) , irq=%d\n",
	       dev->minor,
	       pci6208_boards[i].name,
	       pci_dev->bus->number,
	       PCI_SLOT(pci_dev->devfn),
	       PCI_FUNC(pci_dev->devfn), pci_dev->irq);

	/*  TODO: Warn about non-tested boards. */
	/* switch(board->device_id) */
	/* { */
	/* }; */

	devpriv->pci_dev = pci_dev;

	return 0;
}

static int
pci6208_pci_setup(struct pci_dev *pci_dev, unsigned long *io_base_ptr,
		  int dev_minor)
{
	unsigned long io_base, io_range, lcr_io_base, lcr_io_range;

	/*  Enable PCI device and request regions */
	if (comedi_pci_enable(pci_dev, PCI6208_DRIVER_NAME) < 0) {
		printk(KERN_ERR "comedi%d: Failed to enable PCI device "
			"and request regions\n",
			dev_minor);
		return -EIO;
	}
	/* Read local configuration register
	 * base address [PCI_BASE_ADDRESS #1].
	 */
	lcr_io_base = pci_resource_start(pci_dev, 1);
	lcr_io_range = pci_resource_len(pci_dev, 1);

	printk(KERN_INFO "comedi%d: local config registers at address"
			" 0x%4lx [0x%4lx]\n",
			dev_minor, lcr_io_base, lcr_io_range);

	/*  Read PCI6208 register base address [PCI_BASE_ADDRESS #2]. */
	io_base = pci_resource_start(pci_dev, 2);
	io_range = pci_resource_end(pci_dev, 2) - io_base + 1;

	printk("comedi%d: 6208 registers at address 0x%4lx [0x%4lx]\n",
	       dev_minor, io_base, io_range);

	*io_base_ptr = io_base;
	/* devpriv->io_range = io_range; */
	/* devpriv->is_valid=0; */
	/* devpriv->lcr_io_base=lcr_io_base; */
	/* devpriv->lcr_io_range=lcr_io_range; */

	return 0;
}
