

#ifndef _INDUSTRIAL_IO_H_
#define _INDUSTRIAL_IO_H_

#include <linux/device.h>
#include <linux/cdev.h>
#include "sysfs.h"
#include "chrdev.h"

/* IIO TODO LIST */

/* Event interface flags */
#define IIO_BUSY_BIT_POS 1

struct iio_dev;

static inline s64 iio_get_time_ns(void)
{
	struct timespec ts;
	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	ktime_get_real_ts(&ts);

	return timespec_to_ns(&ts);
}

void iio_add_event_to_list(struct iio_event_handler_list *el,
			   struct list_head *head);

void iio_remove_event_from_list(struct iio_event_handler_list *el,
				struct list_head *head);

/* Device operating modes */
#define INDIO_DIRECT_MODE		0x01
#define INDIO_RING_TRIGGERED		0x02
#define INDIO_RING_HARDWARE_BUFFER	0x08

#define INDIO_ALL_RING_MODES (INDIO_RING_TRIGGERED | INDIO_RING_HARDWARE_BUFFER)


struct iio_dev {
	int				id;
	void				*dev_data;
	int				modes;
	int				currentmode;
	struct device			dev;
	const struct attribute_group	*attrs;
	struct module			*driver_module;

	int				num_interrupt_lines;
	struct iio_interrupt		**interrupts;
	struct attribute_group		*event_attrs;
	struct attribute_group		*event_conf_attrs;

	struct iio_event_interface	*event_interfaces;

	struct iio_ring_buffer		*ring;
	struct mutex			mlock;

	struct attribute_group		*scan_el_attrs;
	int				scan_count;

	u32				scan_mask;
	u32				*available_scan_masks;
	bool				scan_timestamp;
	struct iio_trigger		*trig;
	struct iio_poll_func		*pollfunc;
};

#define IIO_MAX_SCAN_LENGTH 31

/* note 0 used as error indicator as it doesn't make sense. */
static inline u32 iio_scan_mask_match(u32 *av_masks, u32 mask)
{
	while (*av_masks) {
		if (!(~*av_masks & mask))
			return *av_masks;
		av_masks++;
	}
	return 0;
}

static inline int iio_scan_mask_query(struct iio_dev *dev_info, int bit)
{
	u32 mask;

	if (bit > IIO_MAX_SCAN_LENGTH)
		return -EINVAL;

	if (!dev_info->scan_mask)
		return 0;

	if (dev_info->available_scan_masks)
		mask = iio_scan_mask_match(dev_info->available_scan_masks,
					dev_info->scan_mask);
	else
		mask = dev_info->scan_mask;

	if (!mask)
		return -EINVAL;

	return !!(mask & (1 << bit));
};

static inline int iio_scan_mask_set(struct iio_dev *dev_info, int bit)
{
	u32 mask;
	u32 trialmask = dev_info->scan_mask | (1 << bit);

	if (bit > IIO_MAX_SCAN_LENGTH)
		return -EINVAL;
	if (dev_info->available_scan_masks) {
		mask = iio_scan_mask_match(dev_info->available_scan_masks,
					trialmask);
		if (!mask)
			return -EINVAL;
	}
	dev_info->scan_mask = trialmask;
	dev_info->scan_count++;

	return 0;
};

static inline int iio_scan_mask_clear(struct iio_dev *dev_info, int bit)
{
	if (bit > IIO_MAX_SCAN_LENGTH)
		return -EINVAL;
	dev_info->scan_mask &= ~(1 << bit);
	dev_info->scan_count--;
	return 0;
};

static inline int iio_scan_mask_count_to_right(struct iio_dev *dev_info,
						int bit)
{
	int count = 0;
	int mask = (1 << bit);
	if (bit > IIO_MAX_SCAN_LENGTH)
		return -EINVAL;
	while (mask) {
		mask >>= 1;
		if (mask & dev_info->scan_mask)
			count++;
	}

	return count;
}

int iio_device_register(struct iio_dev *dev_info);

void iio_device_unregister(struct iio_dev *dev_info);

struct iio_interrupt {
	struct iio_dev			*dev_info;
	int				line_number;
	int				id;
	int				irq;
	struct list_head		ev_list;
	spinlock_t			ev_list_lock;
};

#define to_iio_interrupt(i) container_of(i, struct iio_interrupt, ev_list)

int iio_register_interrupt_line(unsigned int			irq,
				struct iio_dev			*dev_info,
				int				line_number,
				unsigned long			type,
				const char			*name);

void iio_unregister_interrupt_line(struct iio_dev *dev_info,
				   int line_number);



int iio_push_event(struct iio_dev *dev_info,
		  int ev_line,
		  int ev_code,
		  s64 timestamp);

struct iio_work_cont {
	struct work_struct	ws;
	struct work_struct	ws_nocheck;
	int			address;
	int			mask;
	void			*st;
};

#define to_iio_work_cont_check(_ws)			\
	container_of(_ws, struct iio_work_cont, ws)

#define to_iio_work_cont_no_check(_ws)				\
	container_of(_ws, struct iio_work_cont, ws_nocheck)

static inline void
iio_init_work_cont(struct iio_work_cont *cont,
		   void (*_checkfunc)(struct work_struct *),
		   void (*_nocheckfunc)(struct work_struct *),
		   int _add, int _mask, void *_st)
{
	INIT_WORK(&(cont)->ws, _checkfunc);
	INIT_WORK(&(cont)->ws_nocheck, _nocheckfunc);
	cont->address = _add;
	cont->mask = _mask;
	cont->st = _st;
}
int __iio_push_event(struct iio_event_interface *ev_int,
		    int ev_code,
		    s64 timestamp,
		    struct iio_shared_ev_pointer*
		    shared_pointer_p);
void __iio_change_event(struct iio_detected_event_list *ev,
			int ev_code,
			s64 timestamp);

int iio_setup_ev_int(struct iio_event_interface *ev_int,
		     const char *name,
		     struct module *owner,
		     struct device *dev);

void iio_free_ev_int(struct iio_event_interface *ev_int);

int iio_allocate_chrdev(struct iio_handler *handler, struct iio_dev *dev_info);
void iio_deallocate_chrdev(struct iio_handler *handler);

#define IIO_SIGNED(a) -(a)
#define IIO_UNSIGNED(a) (a)

extern dev_t iio_devt;
extern struct bus_type iio_bus_type;

static inline void iio_put_device(struct iio_dev *dev)
{
	if (dev)
		put_device(&dev->dev);
};

static inline struct iio_dev *to_iio_dev(struct device *d)
{
	return container_of(d, struct iio_dev, dev);
};

static inline void *iio_dev_get_devdata(struct iio_dev *d)
{
	return d->dev_data;
}

struct iio_dev *iio_allocate_device(void);

void iio_free_device(struct iio_dev *dev);

void iio_put(void);

void iio_get(void);

/* Ring buffer related */
int iio_device_get_chrdev_minor(void);
void iio_device_free_chrdev_minor(int val);

static inline bool iio_ring_enabled(struct iio_dev *dev_info)
{
	return dev_info->currentmode
		& (INDIO_RING_TRIGGERED
		   | INDIO_RING_HARDWARE_BUFFER);
};

struct idr;

int iio_get_new_idr_val(struct idr *this_idr);
void iio_free_idr_val(struct idr *this_idr, int id);
#endif /* _INDUSTRIAL_IO_H_ */
