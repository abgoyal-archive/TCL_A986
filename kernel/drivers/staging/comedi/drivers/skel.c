


#include "../comedidev.h"

#include <linux/pci.h>		/* for PCI devices */

/* Imaginary registers for the imaginary board */

#define SKEL_SIZE 0

#define SKEL_START_AI_CONV	0
#define SKEL_AI_READ		0

struct skel_board {
	const char *name;
	int ai_chans;
	int ai_bits;
	int have_dio;
};

static const struct skel_board skel_boards[] = {
	{
	 .name = "skel-100",
	 .ai_chans = 16,
	 .ai_bits = 12,
	 .have_dio = 1,
	 },
	{
	 .name = "skel-200",
	 .ai_chans = 8,
	 .ai_bits = 16,
	 .have_dio = 0,
	 },
};

#define PCI_VENDOR_ID_SKEL 0xdafe
static DEFINE_PCI_DEVICE_TABLE(skel_pci_table) = {
	{
	PCI_VENDOR_ID_SKEL, 0x0100, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	PCI_VENDOR_ID_SKEL, 0x0200, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0}, {
	0}
};

MODULE_DEVICE_TABLE(pci, skel_pci_table);

#define thisboard ((const struct skel_board *)dev->board_ptr)

struct skel_private {

	int data;

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	/* Used for AO readback */
	unsigned int ao_readback[2];
};

#define devpriv ((struct skel_private *)dev->private)

static int skel_attach(struct comedi_device *dev, struct comedi_devconfig *it);
static int skel_detach(struct comedi_device *dev);
static struct comedi_driver driver_skel = {
	.driver_name = "dummy",
	.module = THIS_MODULE,
	.attach = skel_attach,
	.detach = skel_detach,
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
	.board_name = &skel_boards[0].name,
	.offset = sizeof(struct skel_board),
	.num_names = ARRAY_SIZE(skel_boards),
};

static int skel_ai_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data);
static int skel_ao_winsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data);
static int skel_ao_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data);
static int skel_dio_insn_bits(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      struct comedi_insn *insn, unsigned int *data);
static int skel_dio_insn_config(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn, unsigned int *data);
static int skel_ai_cmdtest(struct comedi_device *dev,
			   struct comedi_subdevice *s, struct comedi_cmd *cmd);
static int skel_ns_to_timer(unsigned int *ns, int round);

static int skel_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	struct comedi_subdevice *s;

	pr_info("comedi%d: skel: ", dev->minor);

	/* dev->board_ptr = skel_probe(dev, it); */

	dev->board_name = thisboard->name;

	if (alloc_private(dev, sizeof(struct skel_private)) < 0)
		return -ENOMEM;

	if (alloc_subdevices(dev, 3) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* dev->read_subdev=s; */
	/* analog input subdevice */
	s->type = COMEDI_SUBD_AI;
	/* we support single-ended (ground) and differential */
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_DIFF;
	s->n_chan = thisboard->ai_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &range_bipolar10;
	s->len_chanlist = 16;	/* This is the maximum chanlist length that
				   the board can handle */
	s->insn_read = skel_ai_rinsn;
	s->do_cmdtest = skel_ai_cmdtest;

	s = dev->subdevices + 1;
	/* analog output subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = 1;
	s->maxdata = 0xffff;
	s->range_table = &range_bipolar5;
	s->insn_write = skel_ao_winsn;
	s->insn_read = skel_ao_rinsn;

	s = dev->subdevices + 2;
	/* digital i/o subdevice */
	if (thisboard->have_dio) {
		s->type = COMEDI_SUBD_DIO;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
		s->n_chan = 16;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = skel_dio_insn_bits;
		s->insn_config = skel_dio_insn_config;
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

	pr_info("attached\n");

	return 0;
}

static int skel_detach(struct comedi_device *dev)
{
	pr_info("comedi%d: skel: remove\n", dev->minor);

	return 0;
}

static int skel_ai_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data)
{
	int n, i;
	unsigned int d;
	unsigned int status;

	/* a typical programming sequence */

	/* write channel to multiplexer */
	/* outw(chan,dev->iobase + SKEL_MUX); */

	/* don't wait for mux to settle */

	/* convert n samples */
	for (n = 0; n < insn->n; n++) {
		/* trigger conversion */
		/* outw(0,dev->iobase + SKEL_CONVERT); */

#define TIMEOUT 100
		/* wait for conversion to end */
		for (i = 0; i < TIMEOUT; i++) {
			status = 1;
			/* status = inb(dev->iobase + SKEL_STATUS); */
			if (status)
				break;
		}
		if (i == TIMEOUT) {
			/* printk() should be used instead of printk()
			 * whenever the code can be called from real-time. */
			pr_info("timeout\n");
			return -ETIMEDOUT;
		}

		/* read data */
		/* d = inw(dev->iobase + SKEL_AI_DATA); */
		d = 0;

		/* mangle the data as necessary */
		d ^= 1 << (thisboard->ai_bits - 1);

		data[n] = d;
	}

	/* return the number of samples read/written */
	return n;
}

static int skel_ai_cmdtest(struct comedi_device *dev,
			   struct comedi_subdevice *s, struct comedi_cmd *cmd)
{
	int err = 0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible
     */

	/* note that mutual compatibility is not an issue here */
	if (cmd->scan_begin_src != TRIG_TIMER &&
	    cmd->scan_begin_src != TRIG_EXT)
		err++;
	if (cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
		err++;
	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
#define MAX_SPEED	10000	/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		if (cmd->scan_begin_arg < MAX_SPEED) {
			cmd->scan_begin_arg = MAX_SPEED;
			err++;
		}
		if (cmd->scan_begin_arg > MIN_SPEED) {
			cmd->scan_begin_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if (cmd->scan_begin_arg > 9) {
			cmd->scan_begin_arg = 9;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER) {
		if (cmd->convert_arg < MAX_SPEED) {
			cmd->convert_arg = MAX_SPEED;
			err++;
		}
		if (cmd->convert_arg > MIN_SPEED) {
			cmd->convert_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* see above */
		if (cmd->convert_arg > 9) {
			cmd->convert_arg = 9;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		if (cmd->stop_arg > 0x00ffffff) {
			cmd->stop_arg = 0x00ffffff;
			err++;
		}
	} else {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		tmp = cmd->scan_begin_arg;
		skel_ns_to_timer(&cmd->scan_begin_arg,
				 cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->scan_begin_arg)
			err++;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		skel_ns_to_timer(&cmd->convert_arg,
				 cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			err++;
		if (cmd->scan_begin_src == TRIG_TIMER &&
		    cmd->scan_begin_arg <
		    cmd->convert_arg * cmd->scan_end_arg) {
			cmd->scan_begin_arg =
			    cmd->convert_arg * cmd->scan_end_arg;
			err++;
		}
	}

	if (err)
		return 4;

	return 0;
}

static int skel_ns_to_timer(unsigned int *ns, int round)
{
	/* trivial timer */
	/* if your timing is done through two cascaded timers, the
	 * i8253_cascade_ns_to_timer() function in 8253.h can be
	 * very helpful.  There are also i8254_load() and i8254_mm_load()
	 * which can be used to load values into the ubiquitous 8254 counters
	 */

	return *ns;
}

static int skel_ao_winsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	pr_info("skel_ao_winsn\n");
	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for (i = 0; i < insn->n; i++) {
		/* a typical programming sequence */
		/* outw(data[i],dev->iobase + SKEL_DA0 + chan); */
		devpriv->ao_readback[chan] = data[i];
	}

	/* return the number of samples read/written */
	return i;
}

static int skel_ao_rinsn(struct comedi_device *dev, struct comedi_subdevice *s,
			 struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

static int skel_dio_insn_bits(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      struct comedi_insn *insn, unsigned int *data)
{
	if (insn->n != 2)
		return -EINVAL;

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	if (data[0]) {
		s->state &= ~data[0];
		s->state |= data[0] & data[1];
		/* Write out the new digital output lines */
		/* outw(s->state,dev->iobase + SKEL_DIO); */
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	/* data[1]=inw(dev->iobase + SKEL_DIO); */
	/* or we could just return the software copy of the output values if
	 * it was a purely digital output subdevice */
	/* data[1]=s->state; */

	return 2;
}

static int skel_dio_insn_config(struct comedi_device *dev,
				struct comedi_subdevice *s,
				struct comedi_insn *insn, unsigned int *data)
{
	int chan = CR_CHAN(insn->chanspec);

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */
	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= 1 << chan;
		break;
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~(1 << chan);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] =
		    (s->io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}
	/* outw(s->io_bits,dev->iobase + SKEL_DIO_CONFIG); */

	return insn->n;
}

COMEDI_INITCLEANUP(driver_skel);
/* COMEDI_PCI_INITCLEANUP(driver_skel, skel_pci_table) */
