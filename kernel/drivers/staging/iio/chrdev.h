

#ifndef _IIO_CHRDEV_H_
#define _IIO_CHRDEV_H_
struct iio_dev;

struct iio_handler {
	struct cdev	chrdev;
	int		id;
	unsigned long	flags;
	void		*private;
};

#define iio_cdev_to_handler(cd)				\
	container_of(cd, struct iio_handler, chrdev)

struct iio_event_data {
	int	id;
	s64	timestamp;
};

struct iio_detected_event_list {
	struct list_head		list;
	struct iio_event_data		ev;
	struct iio_shared_ev_pointer	*shared_pointer;
};
struct iio_shared_ev_pointer {
	struct iio_detected_event_list	*ev_p;
	spinlock_t			lock;
};

struct iio_event_interface {
	struct device				dev;
	struct iio_handler			handler;
	wait_queue_head_t			wait;
	struct mutex				event_list_lock;
	struct iio_detected_event_list		det_events;
	int					max_events;
	int					current_events;
	int					id;
	struct iio_chrdev_minor_attr		attr;
	struct module				*owner;
	void					*private;
	char					_name[35];
	char					_attrname[20];
};

struct iio_event_handler_list {
	struct list_head	list;
	int			refcount;
	struct mutex		exist_lock;
	int (*handler)(struct iio_dev *dev_info, int index, s64 timestamp,
		       int no_test);
};

#endif
