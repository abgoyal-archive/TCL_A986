

#ifndef _IIO_RING_GENERIC_H_
#define _IIO_RING_GENERIC_H_
#include "iio.h"

struct iio_handler;
struct iio_ring_buffer;
struct iio_dev;

int iio_push_ring_event(struct iio_ring_buffer *ring_buf,
			int event_code,
			s64 timestamp);
int iio_push_or_escallate_ring_event(struct iio_ring_buffer *ring_buf,
				     int event_code,
				     s64 timestamp);

struct iio_ring_access_funcs {
	void (*mark_in_use)(struct iio_ring_buffer *ring);
	void (*unmark_in_use)(struct iio_ring_buffer *ring);

	int (*store_to)(struct iio_ring_buffer *ring, u8 *data, s64 timestamp);
	int (*read_last)(struct iio_ring_buffer *ring, u8 *data);
	int (*rip_lots)(struct iio_ring_buffer *ring,
			size_t count,
			u8 **data,
			int *dead_offset);

	int (*mark_param_change)(struct iio_ring_buffer *ring);
	int (*request_update)(struct iio_ring_buffer *ring);

	int (*get_bpd)(struct iio_ring_buffer *ring);
	int (*set_bpd)(struct iio_ring_buffer *ring, size_t bpd);
	int (*get_length)(struct iio_ring_buffer *ring);
	int (*set_length)(struct iio_ring_buffer *ring, int length);

	int (*is_enabled)(struct iio_ring_buffer *ring);
	int (*enable)(struct iio_ring_buffer *ring);
};

struct iio_ring_buffer {
	struct device dev;
	struct device access_dev;
	struct iio_dev *indio_dev;
	struct module *owner;
	int				id;
	int				access_id;
	int				length;
	int				bpd;
	int				loopcount;
	struct iio_handler		access_handler;
	struct iio_event_interface	ev_int;
	struct iio_shared_ev_pointer	shared_ev_pointer;
	struct iio_ring_access_funcs	access;
	int				(*preenable)(struct iio_dev *);
	int				(*postenable)(struct iio_dev *);
	int				(*predisable)(struct iio_dev *);
	int				(*postdisable)(struct iio_dev *);

};
void iio_ring_buffer_init(struct iio_ring_buffer *ring,
			  struct iio_dev *dev_info);

static inline void __iio_update_ring_buffer(struct iio_ring_buffer *ring,
					    int bytes_per_datum, int length)
{
	ring->bpd = bytes_per_datum;
	ring->length = length;
	ring->loopcount = 0;
}

struct iio_scan_el {
	struct device_attribute		dev_attr;
	unsigned int			number;
	int				bit_count;
	unsigned int			label;

	int (*set_state)(struct iio_scan_el *scanel,
			 struct iio_dev *dev_info,
			 bool state);
};

#define to_iio_scan_el(_dev_attr)				\
	container_of(_dev_attr, struct iio_scan_el, dev_attr);

ssize_t iio_scan_el_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t len);
ssize_t iio_scan_el_show(struct device *dev, struct device_attribute *attr,
			 char *buf);

ssize_t iio_scan_el_ts_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len);

ssize_t iio_scan_el_ts_show(struct device *dev, struct device_attribute *attr,
			    char *buf);
#define IIO_SCAN_EL_C(_name, _number, _bits, _label, _controlfunc)	\
	struct iio_scan_el iio_scan_el_##_name = {			\
		.dev_attr = __ATTR(_number##_##_name##_en,		\
				   S_IRUGO | S_IWUSR,			\
				   iio_scan_el_show,			\
				   iio_scan_el_store),			\
		.number =  _number,					\
		.bit_count = _bits,					\
		.label = _label,					\
		.set_state = _controlfunc,				\
	}

#define IIO_SCAN_NAMED_EL_C(_name, _string, _number, _bits, _label, _cf) \
	struct iio_scan_el iio_scan_el_##_name = {			\
		.dev_attr = __ATTR(_number##_##_string##_en,		\
				   S_IRUGO | S_IWUSR,			\
				   iio_scan_el_show,			\
				   iio_scan_el_store),			\
		.number =  _number,					\
		.bit_count = _bits,					\
		.label = _label,					\
		.set_state = _cf,					\
	}

#define IIO_SCAN_EL_TIMESTAMP(number)				\
	struct iio_scan_el iio_scan_el_timestamp = {		\
		.dev_attr = __ATTR(number##_timestamp_en,	\
				   S_IRUGO | S_IWUSR,		\
				   iio_scan_el_ts_show,		\
				   iio_scan_el_ts_store),	\
	}

static inline void iio_put_ring_buffer(struct iio_ring_buffer *ring)
{
	put_device(&ring->dev);
};

#define to_iio_ring_buffer(d)			\
	container_of(d, struct iio_ring_buffer, dev)
#define access_dev_to_iio_ring_buffer(d)			\
	container_of(d, struct iio_ring_buffer, access_dev)
int iio_ring_buffer_register(struct iio_ring_buffer *ring, int id);
void iio_ring_buffer_unregister(struct iio_ring_buffer *ring);

ssize_t iio_read_ring_length(struct device *dev,
			     struct device_attribute *attr,
			     char *buf);
ssize_t iio_write_ring_length(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t len);
ssize_t iio_read_ring_bps(struct device *dev,
			  struct device_attribute *attr,
			  char *buf);
ssize_t iio_store_ring_enable(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t len);
ssize_t iio_show_ring_enable(struct device *dev,
			     struct device_attribute *attr,
			     char *buf);
#define IIO_RING_LENGTH_ATTR DEVICE_ATTR(length, S_IRUGO | S_IWUSR,	\
					 iio_read_ring_length,		\
					 iio_write_ring_length)
#define IIO_RING_BPS_ATTR DEVICE_ATTR(bps, S_IRUGO | S_IWUSR,	\
				      iio_read_ring_bps, NULL)
#define IIO_RING_ENABLE_ATTR DEVICE_ATTR(ring_enable, S_IRUGO | S_IWUSR, \
					 iio_show_ring_enable,		\
					 iio_store_ring_enable)

#endif /* _IIO_RING_GENERIC_H_ */
