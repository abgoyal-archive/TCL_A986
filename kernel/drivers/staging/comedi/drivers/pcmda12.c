

#include "../comedidev.h"

#include <linux/pci.h>		/* for PCI devices */

#define SDEV_NO ((int)(s - dev->subdevices))
#define CHANS 8
#define IOSIZE 16
#define LSB(x) ((unsigned char)((x) & 0xff))
#define MSB(x) ((unsigned char)((((unsigned short)(x))>>8) & 0xff))
#define LSB_PORT(chan) (dev->iobase + (chan)*2)
#define MSB_PORT(chan) (LSB_PORT(chan)+1)
#define BITS 12

struct pcmda12_board {
	const char *name;
};

static const struct comedi_lrange pcmda12_ranges = {
	3,
	{
	 UNI_RANGE(5), UNI_RANGE(10), BIP_RANGE(5)
	 }
};

static const struct pcmda12_board pcmda12_boards[] = {
	{
	 .name = "pcmda12",
	 },
};

#define thisboard ((const struct pcmda12_board *)dev->board_ptr)

struct pcmda12_private {

	unsigned int ao_readback[CHANS];
	int simultaneous_xfer_mode;
};

#define devpriv ((struct pcmda12_private *)(dev->private))

static int pcmda12_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it);
static int pcmda12_detach(struct comedi_device *dev);

static void zero_chans(struct comedi_device *dev);

static struct comedi_driver driver = {
	.driver_name = "pcmda12",
	.module = THIS_MODULE,
	.attach = pcmda12_attach,
	.detach = pcmda12_detach,
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in pcmda12_boards[] above.  Note that the element 'name'
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
	.board_name = &pcmda12_boards[0].name,
	.offset = sizeof(struct pcmda12_board),
	.num_names = ARRAY_SIZE(pcmda12_boards),
};

static int ao_winsn(struct comedi_device *dev, struct comedi_subdevice *s,
		    struct comedi_insn *insn, unsigned int *data);
static int ao_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
		    struct comedi_insn *insn, unsigned int *data);

static int pcmda12_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;
	unsigned long iobase;

	iobase = it->options[0];
	printk("comedi%d: %s: io: %lx %s ", dev->minor, driver.driver_name,
	       iobase, it->options[1] ? "simultaneous xfer mode enabled" : "");

	if (!request_region(iobase, IOSIZE, driver.driver_name)) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	dev->iobase = iobase;

	dev->board_name = thisboard->name;

	if (alloc_private(dev, sizeof(struct pcmda12_private)) < 0) {
		printk("cannot allocate private data structure\n");
		return -ENOMEM;
	}

	devpriv->simultaneous_xfer_mode = it->options[1];

	/*
	 * Allocate the subdevice structures.  alloc_subdevice() is a
	 * convenient macro defined in comedidev.h.
	 *
	 * Allocate 2 subdevs (32 + 16 DIO lines) or 3 32 DIO subdevs for the
	 * 96-channel version of the board.
	 */
	if (alloc_subdevices(dev, 1) < 0) {
		printk("cannot allocate subdevice data structures\n");
		return -ENOMEM;
	}

	s = dev->subdevices;
	s->private = NULL;
	s->maxdata = (0x1 << BITS) - 1;
	s->range_table = &pcmda12_ranges;
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = CHANS;
	s->insn_write = &ao_winsn;
	s->insn_read = &ao_rinsn;

	zero_chans(dev);	/* clear out all the registers, basically */

	printk("attached\n");

	return 1;
}

static int pcmda12_detach(struct comedi_device *dev)
{
	printk("comedi%d: %s: remove\n", dev->minor, driver.driver_name);
	if (dev->iobase)
		release_region(dev->iobase, IOSIZE);
	return 0;
}

static void zero_chans(struct comedi_device *dev)
{				/* sets up an
				   ASIC chip to defaults */
	int i;
	for (i = 0; i < CHANS; ++i) {
/*      /\* do this as one instruction?? *\/ */
/*      outw(0, LSB_PORT(chan)); */
		outb(0, LSB_PORT(i));
		outb(0, MSB_PORT(i));
	}
	inb(LSB_PORT(0));	/* update chans. */
}

static int ao_winsn(struct comedi_device *dev, struct comedi_subdevice *s,
		    struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for (i = 0; i < insn->n; ++i) {

/*      /\* do this as one instruction?? *\/ */
/*      outw(data[i], LSB_PORT(chan)); */

		/* Need to do this as two instructions due to 8-bit bus?? */
		/*  first, load the low byte */
		outb(LSB(data[i]), LSB_PORT(chan));
		/*  next, write the high byte */
		outb(MSB(data[i]), MSB_PORT(chan));

		/* save shadow register */
		devpriv->ao_readback[chan] = data[i];

		if (!devpriv->simultaneous_xfer_mode)
			inb(LSB_PORT(chan));
	}

	/* return the number of samples written */
	return i;
}

static int ao_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
		    struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++) {
		if (devpriv->simultaneous_xfer_mode)
			inb(LSB_PORT(chan));
		/* read back shadow register */
		data[i] = devpriv->ao_readback[chan];
	}

	return i;
}

COMEDI_INITCLEANUP(driver);
