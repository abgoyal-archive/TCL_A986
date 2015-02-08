


#include <linux/string.h>
#include <linux/slab.h>
#include "../comedi.h"
#include "../comedilib.h"
#include "../comedidev.h"

/* The maxiumum number of channels per subdevice. */
#define MAX_CHANS 256

#define MODULE_NAME "comedi_bond"
MODULE_LICENSE("GPL");
#ifndef STR
#  define STR1(x) #x
#  define STR(x) STR1(x)
#endif

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "If true, print extra cryptic debugging output useful"
		 "only to developers.");

#define LOG_MSG(x...) printk(KERN_INFO MODULE_NAME": "x)
#define DEBUG(x...)							\
	do {								\
		if (debug)						\
			printk(KERN_DEBUG MODULE_NAME": DEBUG: "x);	\
	} while (0)
#define WARNING(x...)  printk(KERN_WARNING MODULE_NAME ": WARNING: "x)
#define ERROR(x...)  printk(KERN_ERR MODULE_NAME ": INTERNAL ERROR: "x)
MODULE_AUTHOR("Calin A. Culianu");
MODULE_DESCRIPTION(MODULE_NAME "A driver for COMEDI to bond multiple COMEDI "
		   "devices together as one.  In the words of John Lennon: "
		   "'And the world will live as one...'");

struct BondingBoard {
	const char *name;
};

static const struct BondingBoard bondingBoards[] = {
	{
	 .name = MODULE_NAME,
	 },
};

#define thisboard ((const struct BondingBoard *)dev->board_ptr)

struct BondedDevice {
	struct comedi_device *dev;
	unsigned minor;
	unsigned subdev;
	unsigned subdev_type;
	unsigned nchans;
	unsigned chanid_offset;	/* The offset into our unified linear
				   channel-id's of chanid 0 on this
				   subdevice. */
};

struct Private {
# define MAX_BOARD_NAME 256
	char name[MAX_BOARD_NAME];
	struct BondedDevice **devs;
	unsigned ndevs;
	struct BondedDevice *chanIdDevMap[MAX_CHANS];
	unsigned nchans;
};

#define devpriv ((struct Private *)dev->private)

static int bonding_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it);
static int bonding_detach(struct comedi_device *dev);
/** Build Private array of all devices.. */
static int doDevConfig(struct comedi_device *dev, struct comedi_devconfig *it);
static void doDevUnconfig(struct comedi_device *dev);
static void *Realloc(const void *ptr, size_t len, size_t old_len);

static struct comedi_driver driver_bonding = {
	.driver_name = MODULE_NAME,
	.module = THIS_MODULE,
	.attach = bonding_attach,
	.detach = bonding_detach,
	/* It is not necessary to implement the following members if you are
	 * writing a driver for a ISA PnP or PCI card */
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in skel_boards[] above.  Note that the element 'name'
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
	.board_name = &bondingBoards[0].name,
	.offset = sizeof(struct BondingBoard),
	.num_names = ARRAY_SIZE(bondingBoards),
};

static int bonding_dio_insn_bits(struct comedi_device *dev,
				 struct comedi_subdevice *s,
				 struct comedi_insn *insn, unsigned int *data);
static int bonding_dio_insn_config(struct comedi_device *dev,
				   struct comedi_subdevice *s,
				   struct comedi_insn *insn,
				   unsigned int *data);

static int bonding_attach(struct comedi_device *dev,
			  struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;

	LOG_MSG("comedi%d\n", dev->minor);

	/*
	 * Allocate the private structure area.  alloc_private() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_private(dev, sizeof(struct Private)) < 0)
		return -ENOMEM;

	/*
	 * Setup our bonding from config params.. sets up our Private struct..
	 */
	if (!doDevConfig(dev, it))
		return -EINVAL;

	/*
	 * Initialize dev->board_name.  Note that we can use the "thisboard"
	 * macro now, since we just initialized it in the last line.
	 */
	dev->board_name = devpriv->name;

	/*
	 * Allocate the subdevice structures.  alloc_subdevice() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_subdevices(dev, 1) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = devpriv->nchans;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = bonding_dio_insn_bits;
	s->insn_config = bonding_dio_insn_config;

	LOG_MSG("attached with %u DIO channels coming from %u different "
		"subdevices all bonded together.  "
		"John Lennon would be proud!\n",
		devpriv->nchans, devpriv->ndevs);

	return 1;
}

static int bonding_detach(struct comedi_device *dev)
{
	LOG_MSG("comedi%d: remove\n", dev->minor);
	doDevUnconfig(dev);
	return 0;
}

static int bonding_dio_insn_bits(struct comedi_device *dev,
				 struct comedi_subdevice *s,
				 struct comedi_insn *insn, unsigned int *data)
{
#define LSAMPL_BITS (sizeof(unsigned int)*8)
	unsigned nchans = LSAMPL_BITS, num_done = 0, i;
	if (insn->n != 2)
		return -EINVAL;

	if (devpriv->nchans < nchans)
		nchans = devpriv->nchans;

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	for (i = 0; num_done < nchans && i < devpriv->ndevs; ++i) {
		struct BondedDevice *bdev = devpriv->devs[i];
		/* Grab the channel mask and data of only the bits corresponding
		   to this subdevice.. need to shift them to zero position of
		   course. */
		/* Bits corresponding to this subdev. */
		unsigned int subdevMask = ((1 << bdev->nchans) - 1);
		unsigned int writeMask, dataBits;

		/* Argh, we have >= LSAMPL_BITS chans.. take all bits */
		if (bdev->nchans >= LSAMPL_BITS)
			subdevMask = (unsigned int)(-1);

		writeMask = (data[0] >> num_done) & subdevMask;
		dataBits = (data[1] >> num_done) & subdevMask;

		/* Read/Write the new digital lines */
		if (comedi_dio_bitfield(bdev->dev, bdev->subdev, writeMask,
					&dataBits) != 2)
			return -EINVAL;

		/* Make room for the new bits in data[1], the return value */
		data[1] &= ~(subdevMask << num_done);
		/* Put the bits in the return value */
		data[1] |= (dataBits & subdevMask) << num_done;
		/* Save the new bits to the saved state.. */
		s->state = data[1];

		num_done += bdev->nchans;
	}

	return insn->n;
}

static int bonding_dio_insn_config(struct comedi_device *dev,
				   struct comedi_subdevice *s,
				   struct comedi_insn *insn, unsigned int *data)
{
	int chan = CR_CHAN(insn->chanspec), ret, io_bits = s->io_bits;
	unsigned int io;
	struct BondedDevice *bdev;

	if (chan < 0 || chan >= devpriv->nchans)
		return -EINVAL;
	bdev = devpriv->chanIdDevMap[chan];

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */
	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
		io = COMEDI_OUTPUT;	/* is this really necessary? */
		io_bits |= 1 << chan;
		break;
	case INSN_CONFIG_DIO_INPUT:
		io = COMEDI_INPUT;	/* is this really necessary? */
		io_bits &= ~(1 << chan);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] =
		    (io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}
	/* 'real' channel id for this subdev.. */
	chan -= bdev->chanid_offset;
	ret = comedi_dio_config(bdev->dev, bdev->subdev, chan, io);
	if (ret != 1)
		return -EINVAL;
	/* Finally, save the new io_bits values since we didn't get
	   an error above. */
	s->io_bits = io_bits;
	return insn->n;
}

static void *Realloc(const void *oldmem, size_t newlen, size_t oldlen)
{
	void *newmem = kmalloc(newlen, GFP_KERNEL);

	if (newmem && oldmem)
		memcpy(newmem, oldmem, min(oldlen, newlen));
	kfree(oldmem);
	return newmem;
}

static int doDevConfig(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int i;
	struct comedi_device *devs_opened[COMEDI_NUM_BOARD_MINORS];

	memset(devs_opened, 0, sizeof(devs_opened));
	devpriv->name[0] = 0;;
	/* Loop through all comedi devices specified on the command-line,
	   building our device list */
	for (i = 0; i < COMEDI_NDEVCONFOPTS && (!i || it->options[i]); ++i) {
		char file[] = "/dev/comediXXXXXX";
		int minor = it->options[i];
		struct comedi_device *d;
		int sdev = -1, nchans, tmp;
		struct BondedDevice *bdev = NULL;

		if (minor < 0 || minor >= COMEDI_NUM_BOARD_MINORS) {
			ERROR("Minor %d is invalid!\n", minor);
			return 0;
		}
		if (minor == dev->minor) {
			ERROR("Cannot bond this driver to itself!\n");
			return 0;
		}
		if (devs_opened[minor]) {
			ERROR("Minor %d specified more than once!\n", minor);
			return 0;
		}

		snprintf(file, sizeof(file), "/dev/comedi%u", minor);
		file[sizeof(file) - 1] = 0;

		d = devs_opened[minor] = comedi_open(file);

		if (!d) {
			ERROR("Minor %u could not be opened\n", minor);
			return 0;
		}

		/* Do DIO, as that's all we support now.. */
		while ((sdev = comedi_find_subdevice_by_type(d, COMEDI_SUBD_DIO,
							     sdev + 1)) > -1) {
			nchans = comedi_get_n_channels(d, sdev);
			if (nchans <= 0) {
				ERROR("comedi_get_n_channels() returned %d "
				      "on minor %u subdev %d!\n",
				      nchans, minor, sdev);
				return 0;
			}
			bdev = kmalloc(sizeof(*bdev), GFP_KERNEL);
			if (!bdev) {
				ERROR("Out of memory.\n");
				return 0;
			}
			bdev->dev = d;
			bdev->minor = minor;
			bdev->subdev = sdev;
			bdev->subdev_type = COMEDI_SUBD_DIO;
			bdev->nchans = nchans;
			bdev->chanid_offset = devpriv->nchans;

			/* map channel id's to BondedDevice * pointer.. */
			while (nchans--)
				devpriv->chanIdDevMap[devpriv->nchans++] = bdev;

			/* Now put bdev pointer at end of devpriv->devs array
			 * list.. */

			/* ergh.. ugly.. we need to realloc :(  */
			tmp = devpriv->ndevs * sizeof(bdev);
			devpriv->devs =
			    Realloc(devpriv->devs,
				    ++devpriv->ndevs * sizeof(bdev), tmp);
			if (!devpriv->devs) {
				ERROR("Could not allocate memory. "
				      "Out of memory?");
				return 0;
			}

			devpriv->devs[devpriv->ndevs - 1] = bdev;
			{
	/** Append dev:subdev to devpriv->name */
				char buf[20];
				int left =
				    MAX_BOARD_NAME - strlen(devpriv->name) - 1;
				snprintf(buf, sizeof(buf), "%d:%d ", dev->minor,
					 bdev->subdev);
				buf[sizeof(buf) - 1] = 0;
				strncat(devpriv->name, buf, left);
			}

		}
	}

	if (!devpriv->nchans) {
		ERROR("No channels found!\n");
		return 0;
	}

	return 1;
}

static void doDevUnconfig(struct comedi_device *dev)
{
	unsigned long devs_closed = 0;

	if (devpriv) {
		while (devpriv->ndevs-- && devpriv->devs) {
			struct BondedDevice *bdev;

			bdev = devpriv->devs[devpriv->ndevs];
			if (!bdev)
				continue;
			if (!(devs_closed & (0x1 << bdev->minor))) {
				comedi_close(bdev->dev);
				devs_closed |= (0x1 << bdev->minor);
			}
			kfree(bdev);
		}
		kfree(devpriv->devs);
		devpriv->devs = NULL;
		kfree(devpriv);
		dev->private = NULL;
	}
}

static int __init init(void)
{
	return comedi_driver_register(&driver_bonding);
}

static void __exit cleanup(void)
{
	comedi_driver_unregister(&driver_bonding);
}

module_init(init);
module_exit(cleanup);
