

#ifndef _COMEDI_FC_H
#define _COMEDI_FC_H

#include "../comedidev.h"

/* Writes an array of data points to comedi's buffer */
extern unsigned int cfc_write_array_to_buffer(struct comedi_subdevice *subd,
					      void *data,
					      unsigned int num_bytes);

static inline unsigned int cfc_write_to_buffer(struct comedi_subdevice *subd,
					       short data)
{
	return cfc_write_array_to_buffer(subd, &data, sizeof(data));
};

static inline unsigned int cfc_write_long_to_buffer(struct comedi_subdevice
						    *subd, unsigned int data)
{
	return cfc_write_array_to_buffer(subd, &data, sizeof(data));
};

extern unsigned int cfc_read_array_from_buffer(struct comedi_subdevice *subd,
					       void *data,
					       unsigned int num_bytes);

extern unsigned int cfc_handle_events(struct comedi_device *dev,
				      struct comedi_subdevice *subd);

static inline unsigned int cfc_bytes_per_scan(struct comedi_subdevice *subd)
{
	int num_samples;
	int bits_per_sample;

	switch (subd->type) {
	case COMEDI_SUBD_DI:
	case COMEDI_SUBD_DO:
	case COMEDI_SUBD_DIO:
		bits_per_sample = 8 * bytes_per_sample(subd);
		num_samples = (subd->async->cmd.chanlist_len +
			       bits_per_sample - 1) / bits_per_sample;
		break;
	default:
		num_samples = subd->async->cmd.chanlist_len;
		break;
	}
	return num_samples * bytes_per_sample(subd);
}

#endif /* _COMEDI_FC_H */
