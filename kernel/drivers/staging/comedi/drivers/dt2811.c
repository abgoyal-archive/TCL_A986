

#include <linux/interrupt.h>
#include "../comedidev.h"

#include <linux/ioport.h>

static const char *driver_name = "dt2811";

static const struct comedi_lrange range_dt2811_pgh_ai_5_unipolar = {
	4, {
		RANGE(0, 5),
		RANGE(0, 2.5),
		RANGE(0, 1.25),
		RANGE(0, 0.625)
	}
};

static const struct comedi_lrange range_dt2811_pgh_ai_2_5_bipolar = {
	4, {
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
		RANGE(-0.625, 0.625),
		RANGE(-0.3125, 0.3125)
	}
};

static const struct comedi_lrange range_dt2811_pgh_ai_5_bipolar = {
	4, {
		RANGE(-5, 5),
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
		RANGE(-0.625, 0.625)
	}
};

static const struct comedi_lrange range_dt2811_pgl_ai_5_unipolar = {
	4, {
		RANGE(0, 5),
		RANGE(0, 0.5),
		RANGE(0, 0.05),
		RANGE(0, 0.01)
	}
};

static const struct comedi_lrange range_dt2811_pgl_ai_2_5_bipolar = {
	4, {
		RANGE(-2.5, 2.5),
		RANGE(-0.25, 0.25),
		RANGE(-0.025, 0.025),
		RANGE(-0.005, 0.005)
	}
};

static const struct comedi_lrange range_dt2811_pgl_ai_5_bipolar = {
	4, {
		RANGE(-5, 5),
		RANGE(-0.5, 0.5),
		RANGE(-0.05, 0.05),
		RANGE(-0.01, 0.01)
	}
};


#define TIMEOUT 10000

#define DT2811_SIZE 8

#define DT2811_ADCSR 0
#define DT2811_ADGCR 1
#define DT2811_ADDATLO 2
#define DT2811_ADDATHI 3
#define DT2811_DADAT0LO 2
#define DT2811_DADAT0HI 3
#define DT2811_DADAT1LO 4
#define DT2811_DADAT1HI 5
#define DT2811_DIO 6
#define DT2811_TMRCTR 7


/* ADCSR */

#define DT2811_ADDONE   0x80
#define DT2811_ADERROR  0x40
#define DT2811_ADBUSY   0x20
#define DT2811_CLRERROR 0x10
#define DT2811_INTENB   0x04
#define DT2811_ADMODE   0x03

struct dt2811_board {

	const char *name;
	const struct comedi_lrange *bip_5;
	const struct comedi_lrange *bip_2_5;
	const struct comedi_lrange *unip_5;
};

static const struct dt2811_board boardtypes[] = {
	{"dt2811-pgh",
	 &range_dt2811_pgh_ai_5_bipolar,
	 &range_dt2811_pgh_ai_2_5_bipolar,
	 &range_dt2811_pgh_ai_5_unipolar,
	 },
	{"dt2811-pgl",
	 &range_dt2811_pgl_ai_5_bipolar,
	 &range_dt2811_pgl_ai_2_5_bipolar,
	 &range_dt2811_pgl_ai_5_unipolar,
	 },
};

#define this_board ((const struct dt2811_board *)dev->board_ptr)

static int dt2811_attach(struct comedi_device *dev,
			 struct comedi_devconfig *it);
static int dt2811_detach(struct comedi_device *dev);
static struct comedi_driver driver_dt2811 = {
	.driver_name = "dt2811",
	.module = THIS_MODULE,
	.attach = dt2811_attach,
	.detach = dt2811_detach,
	.board_name = &boardtypes[0].name,
	.num_names = ARRAY_SIZE(boardtypes),
	.offset = sizeof(struct dt2811_board),
};

COMEDI_INITCLEANUP(driver_dt2811);

static int dt2811_ai_insn(struct comedi_device *dev, struct comedi_subdevice *s,
			  struct comedi_insn *insn, unsigned int *data);
static int dt2811_ao_insn(struct comedi_device *dev, struct comedi_subdevice *s,
			  struct comedi_insn *insn, unsigned int *data);
static int dt2811_ao_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data);
static int dt2811_di_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data);
static int dt2811_do_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data);

enum { card_2811_pgh, card_2811_pgl };

struct dt2811_private {
	int ntrig;
	int curadchan;
	enum {
		adc_singleended, adc_diff, adc_pseudo_diff
	} adc_mux;
	enum {
		dac_bipolar_5, dac_bipolar_2_5, dac_unipolar_5
	} dac_range[2];
	const struct comedi_lrange *range_type_list[2];
	unsigned int ao_readback[2];
};

#define devpriv ((struct dt2811_private *)dev->private)

static const struct comedi_lrange *dac_range_types[] = {
	&range_bipolar5,
	&range_bipolar2_5,
	&range_unipolar5
};

#define DT2811_TIMEOUT 5

#if 0
static irqreturn_t dt2811_interrupt(int irq, void *d)
{
	int lo, hi;
	int data;
	struct comedi_device *dev = d;

	if (!dev->attached) {
		comedi_error(dev, "spurious interrupt");
		return IRQ_HANDLED;
	}

	lo = inb(dev->iobase + DT2811_ADDATLO);
	hi = inb(dev->iobase + DT2811_ADDATHI);

	data = lo + (hi << 8);

	if (!(--devpriv->ntrig)) {
		/* how to turn off acquisition */
		s->async->events |= COMEDI_SB_EOA;
	}
	comedi_event(dev, s);
	return IRQ_HANDLED;
}
#endif


static int dt2811_attach(struct comedi_device *dev, struct comedi_devconfig *it)
{
	/* int i, irq; */
	/* unsigned long irqs; */
	/* long flags; */

	int ret;
	struct comedi_subdevice *s;
	unsigned long iobase;

	iobase = it->options[0];

	printk(KERN_INFO "comedi%d: dt2811:base=0x%04lx\n", dev->minor, iobase);

	if (!request_region(iobase, DT2811_SIZE, driver_name)) {
		printk(KERN_ERR "I/O port conflict\n");
		return -EIO;
	}

	dev->iobase = iobase;
	dev->board_name = this_board->name;

#if 0
	outb(0, dev->iobase + DT2811_ADCSR);
	udelay(100);
	i = inb(dev->iobase + DT2811_ADDATLO);
	i = inb(dev->iobase + DT2811_ADDATHI);
#endif

#if 0
	irq = it->options[1];
	if (irq < 0) {
		save_flags(flags);
		sti();
		irqs = probe_irq_on();

		outb(DT2811_CLRERROR | DT2811_INTENB,
		     dev->iobase + DT2811_ADCSR);
		outb(0, dev->iobase + DT2811_ADGCR);

		udelay(100);

		irq = probe_irq_off(irqs);
		restore_flags(flags);

		/*outb(DT2811_CLRERROR|DT2811_INTENB,
			dev->iobase+DT2811_ADCSR);*/

		if (inb(dev->iobase + DT2811_ADCSR) & DT2811_ADERROR)
			printk(KERN_ERR "error probing irq (bad)\n");
		dev->irq = 0;
		if (irq > 0) {
			i = inb(dev->iobase + DT2811_ADDATLO);
			i = inb(dev->iobase + DT2811_ADDATHI);
			printk(KERN_INFO "(irq = %d)\n", irq);
			ret = request_irq(irq, dt2811_interrupt, 0,
					  driver_name, dev);
			if (ret < 0)
				return -EIO;
			dev->irq = irq;
		} else if (irq == 0) {
			printk(KERN_INFO "(no irq)\n");
		} else {
			printk(KERN_ERR "( multiple irq's -- this is bad! )\n");
		}
	}
#endif

	ret = alloc_subdevices(dev, 4);
	if (ret < 0)
		return ret;

	ret = alloc_private(dev, sizeof(struct dt2811_private));
	if (ret < 0)
		return ret;

	switch (it->options[2]) {
	case 0:
		devpriv->adc_mux = adc_singleended;
		break;
	case 1:
		devpriv->adc_mux = adc_diff;
		break;
	case 2:
		devpriv->adc_mux = adc_pseudo_diff;
		break;
	default:
		devpriv->adc_mux = adc_singleended;
		break;
	}
	switch (it->options[4]) {
	case 0:
		devpriv->dac_range[0] = dac_bipolar_5;
		break;
	case 1:
		devpriv->dac_range[0] = dac_bipolar_2_5;
		break;
	case 2:
		devpriv->dac_range[0] = dac_unipolar_5;
		break;
	default:
		devpriv->dac_range[0] = dac_bipolar_5;
		break;
	}
	switch (it->options[5]) {
	case 0:
		devpriv->dac_range[1] = dac_bipolar_5;
		break;
	case 1:
		devpriv->dac_range[1] = dac_bipolar_2_5;
		break;
	case 2:
		devpriv->dac_range[1] = dac_unipolar_5;
		break;
	default:
		devpriv->dac_range[1] = dac_bipolar_5;
		break;
	}

	s = dev->subdevices + 0;
	/* initialize the ADC subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND;
	s->n_chan = devpriv->adc_mux == adc_diff ? 8 : 16;
	s->insn_read = dt2811_ai_insn;
	s->maxdata = 0xfff;
	switch (it->options[3]) {
	case 0:
	default:
		s->range_table = this_board->bip_5;
		break;
	case 1:
		s->range_table = this_board->bip_2_5;
		break;
	case 2:
		s->range_table = this_board->unip_5;
		break;
	}

	s = dev->subdevices + 1;
	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = 2;
	s->insn_write = dt2811_ao_insn;
	s->insn_read = dt2811_ao_insn_read;
	s->maxdata = 0xfff;
	s->range_table_list = devpriv->range_type_list;
	devpriv->range_type_list[0] = dac_range_types[devpriv->dac_range[0]];
	devpriv->range_type_list[1] = dac_range_types[devpriv->dac_range[1]];

	s = dev->subdevices + 2;
	/* di subdevice */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->insn_bits = dt2811_di_insn_bits;
	s->maxdata = 1;
	s->range_table = &range_digital;

	s = dev->subdevices + 3;
	/* do subdevice */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = 8;
	s->insn_bits = dt2811_do_insn_bits;
	s->maxdata = 1;
	s->state = 0;
	s->range_table = &range_digital;

	return 0;
}

static int dt2811_detach(struct comedi_device *dev)
{
	printk(KERN_INFO "comedi%d: dt2811: remove\n", dev->minor);

	if (dev->irq)
		free_irq(dev->irq, dev);
	if (dev->iobase)
		release_region(dev->iobase, DT2811_SIZE);

	return 0;
}

static int dt2811_ai_insn(struct comedi_device *dev, struct comedi_subdevice *s,
			  struct comedi_insn *insn, unsigned int *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int timeout = DT2811_TIMEOUT;
	int i;

	for (i = 0; i < insn->n; i++) {
		outb(chan, dev->iobase + DT2811_ADGCR);

		while (timeout
		       && inb(dev->iobase + DT2811_ADCSR) & DT2811_ADBUSY)
			timeout--;
		if (!timeout)
			return -ETIME;

		data[i] = inb(dev->iobase + DT2811_ADDATLO);
		data[i] |= inb(dev->iobase + DT2811_ADDATHI) << 8;
		data[i] &= 0xfff;
	}

	return i;
}

#if 0
int dt2811_adtrig(kdev_t minor, comedi_adtrig *adtrig)
{
	struct comedi_device *dev = comedi_devices + minor;

	if (adtrig->n < 1)
		return 0;
	dev->curadchan = adtrig->chan;
	switch (dev->i_admode) {
	case COMEDI_MDEMAND:
		dev->ntrig = adtrig->n - 1;
		/* not neccessary */
		/*printk("dt2811: AD soft trigger\n"); */
		/*outb(DT2811_CLRERROR|DT2811_INTENB,
			dev->iobase+DT2811_ADCSR); */
		outb(dev->curadchan, dev->iobase + DT2811_ADGCR);
		do_gettimeofday(&trigtime);
		break;
	case COMEDI_MCONTS:
		dev->ntrig = adtrig->n;
		break;
	}

	return 0;
}
#endif

static int dt2811_ao_insn(struct comedi_device *dev, struct comedi_subdevice *s,
			  struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan;

	chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++) {
		outb(data[i] & 0xff, dev->iobase + DT2811_DADAT0LO + 2 * chan);
		outb((data[i] >> 8) & 0xff,
		     dev->iobase + DT2811_DADAT0HI + 2 * chan);
		devpriv->ao_readback[chan] = data[i];
	}

	return i;
}

static int dt2811_ao_insn_read(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	int i;
	int chan;

	chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

static int dt2811_di_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	if (insn->n != 2)
		return -EINVAL;

	data[1] = inb(dev->iobase + DT2811_DIO);

	return 2;
}

static int dt2811_do_insn_bits(struct comedi_device *dev,
			       struct comedi_subdevice *s,
			       struct comedi_insn *insn, unsigned int *data)
{
	if (insn->n != 2)
		return -EINVAL;

	s->state &= ~data[0];
	s->state |= data[0] & data[1];
	outb(s->state, dev->iobase + DT2811_DIO);

	data[1] = s->state;

	return 2;
}
