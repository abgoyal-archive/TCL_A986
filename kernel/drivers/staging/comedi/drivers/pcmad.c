

#include <linux/interrupt.h>
#include "../comedidev.h"

#include <linux/ioport.h>

#define PCMAD_SIZE		4

#define PCMAD_STATUS		0
#define PCMAD_LSB		1
#define PCMAD_MSB		2
#define PCMAD_CONVERT		1

struct pcmad_board_struct {
	const char *name;
	int n_ai_bits;
};
static const struct pcmad_board_struct pcmad_boards[] = {
	{
	 .name = "pcmad12",
	 .n_ai_bits = 12,
	 },
	{
	 .name = "pcmad16",
	 .n_ai_bits = 16,
	 },
};

#define this_board ((const struct pcmad_board_struct *)(dev->board_ptr))
#define n_pcmad_boards ARRAY_SIZE(pcmad_boards)

struct pcmad_priv_struct {
	int differential;
	int twos_comp;
};
#define devpriv ((struct pcmad_priv_struct *)dev->private)

static int pcmad_attach(struct comedi_device *dev, struct comedi_devconfig *it);
static int pcmad_detach(struct comedi_device *dev);
static struct comedi_driver driver_pcmad = {
	.driver_name = "pcmad",
	.module = THIS_MODULE,
	.attach = pcmad_attach,
	.detach = pcmad_detach,
	.board_name = &pcmad_boards[0].name,
	.num_names = n_pcmad_boards,
	.offset = sizeof(pcmad_boards[0]),
};

COMEDI_INITCLEANUP(driver_pcmad);

#define TIMEOUT	100

static int pcmad_ai_insn_read(struct comedi_device *dev,
			      struct comedi_subdevice *s,
			      struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan;
	int n;

	chan = CR_CHAN(insn->chanspec);

	for (n = 0; n < insn->n; n++) {
		outb(chan, dev->iobase + PCMAD_CONVERT);

		for (i = 0; i < TIMEOUT; i++) {
			if ((inb(dev->iobase + PCMAD_STATUS) & 0x3) == 0x3)
				break;
		}
		data[n] = inb(dev->iobase + PCMAD_LSB);
		data[n] |= (inb(dev->iobase + PCMAD_MSB) << 8);

		if (devpriv->twos_comp)
			data[n] ^= (1 << (this_board->n_ai_bits - 1));
	}

	return n;
}

static int pcmad_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	int ret;
	struct comedi_subdevice *s;
	unsigned long iobase;

	iobase = it->options[0];
	printk(KERN_INFO "comedi%d: pcmad: 0x%04lx ", dev->minor, iobase);
	if (!request_region(iobase, PCMAD_SIZE, "pcmad")) {
		printk(KERN_CONT "I/O port conflict\n");
		return -EIO;
	}
	printk(KERN_CONT "\n");
	dev->iobase = iobase;

	ret = alloc_subdevices(dev, 1);
	if (ret < 0)
		return ret;

	ret = alloc_private(dev, sizeof(struct pcmad_priv_struct));
	if (ret < 0)
		return ret;

	dev->board_name = this_board->name;

	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | AREF_GROUND;
	s->n_chan = 16;		/* XXX */
	s->len_chanlist = 1;
	s->insn_read = pcmad_ai_insn_read;
	s->maxdata = (1 << this_board->n_ai_bits) - 1;
	s->range_table = &range_unknown;

	return 0;
}

static int pcmad_detach(struct comedi_device *dev)
{
	printk(KERN_INFO "comedi%d: pcmad: remove\n", dev->minor);

	if (dev->irq)
		free_irq(dev->irq, dev);

	if (dev->iobase)
		release_region(dev->iobase, PCMAD_SIZE);

	return 0;
}
