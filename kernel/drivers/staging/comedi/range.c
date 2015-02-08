

#include <linux/uaccess.h>
#include "comedidev.h"
#include "internal.h"

const struct comedi_lrange range_bipolar10 = { 1, {BIP_RANGE(10)} };
EXPORT_SYMBOL(range_bipolar10);
const struct comedi_lrange range_bipolar5 = { 1, {BIP_RANGE(5)} };
EXPORT_SYMBOL(range_bipolar5);
const struct comedi_lrange range_bipolar2_5 = { 1, {BIP_RANGE(2.5)} };
EXPORT_SYMBOL(range_bipolar2_5);
const struct comedi_lrange range_unipolar10 = { 1, {UNI_RANGE(10)} };
EXPORT_SYMBOL(range_unipolar10);
const struct comedi_lrange range_unipolar5 = { 1, {UNI_RANGE(5)} };
EXPORT_SYMBOL(range_unipolar5);
const struct comedi_lrange range_unknown = { 1, {{0, 1000000, UNIT_none} } };
EXPORT_SYMBOL(range_unknown);

int do_rangeinfo_ioctl(struct comedi_device *dev,
		       struct comedi_rangeinfo __user *arg)
{
	struct comedi_rangeinfo it;
	int subd, chan;
	const struct comedi_lrange *lr;
	struct comedi_subdevice *s;

	if (copy_from_user(&it, arg, sizeof(struct comedi_rangeinfo)))
		return -EFAULT;
	subd = (it.range_type >> 24) & 0xf;
	chan = (it.range_type >> 16) & 0xff;

	if (!dev->attached)
		return -EINVAL;
	if (subd >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + subd;
	if (s->range_table) {
		lr = s->range_table;
	} else if (s->range_table_list) {
		if (chan >= s->n_chan)
			return -EINVAL;
		lr = s->range_table_list[chan];
	} else {
		return -EINVAL;
	}

	if (RANGE_LENGTH(it.range_type) != lr->length) {
		DPRINTK("wrong length %d should be %d (0x%08x)\n",
			RANGE_LENGTH(it.range_type), lr->length, it.range_type);
		return -EINVAL;
	}

	if (copy_to_user(it.range_ptr, lr->range,
			 sizeof(struct comedi_krange) * lr->length))
		return -EFAULT;

	return 0;
}

static int aref_invalid(struct comedi_subdevice *s, unsigned int chanspec)
{
	unsigned int aref;

	/*  disable reporting invalid arefs... maybe someday */
	return 0;

	aref = CR_AREF(chanspec);
	switch (aref) {
	case AREF_DIFF:
		if (s->subdev_flags & SDF_DIFF)
			return 0;
		break;
	case AREF_COMMON:
		if (s->subdev_flags & SDF_COMMON)
			return 0;
		break;
	case AREF_GROUND:
		if (s->subdev_flags & SDF_GROUND)
			return 0;
		break;
	case AREF_OTHER:
		if (s->subdev_flags & SDF_OTHER)
			return 0;
		break;
	default:
		break;
	}
	DPRINTK("subdevice does not support aref %i", aref);
	return 1;
}

int comedi_check_chanlist(struct comedi_subdevice *s, int n,
			  unsigned int *chanlist)
{
	int i;
	int chan;

	if (s->range_table) {
		for (i = 0; i < n; i++)
			if (CR_CHAN(chanlist[i]) >= s->n_chan ||
			    CR_RANGE(chanlist[i]) >= s->range_table->length
			    || aref_invalid(s, chanlist[i])) {
				printk(KERN_ERR "bad chanlist[%d]=0x%08x "
				       "in_chan=%d range length=%d\n", i,
				       chanlist[i], s->n_chan,
				       s->range_table->length);
				return -EINVAL;
			}
	} else if (s->range_table_list) {
		for (i = 0; i < n; i++) {
			chan = CR_CHAN(chanlist[i]);
			if (chan >= s->n_chan ||
			    CR_RANGE(chanlist[i]) >=
			    s->range_table_list[chan]->length
			    || aref_invalid(s, chanlist[i])) {
				printk(KERN_ERR "bad chanlist[%d]=0x%08x\n",
				       i, chanlist[i]);
				return -EINVAL;
			}
		}
	} else {
		printk(KERN_ERR "comedi: (bug) no range type list!\n");
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(comedi_check_chanlist);
