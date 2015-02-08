
#ifndef _IIO_TRIGGER_H_
#define _IIO_TRIGGER_H_
#define IIO_TRIGGER_NAME_LENGTH 20
#define IIO_TRIGGER_ID_PREFIX "iio:trigger"
#define IIO_TRIGGER_ID_FORMAT IIO_TRIGGER_ID_PREFIX "%d"


struct iio_trigger {
	int				id;
	const char			*name;
	struct device			dev;

	void				*private_data;
	struct list_head		list;
	struct list_head		alloc_list;
	spinlock_t			pollfunc_list_lock;
	struct list_head		pollfunc_list;
	const struct attribute_group	*control_attrs;
	s64				timestamp;
	struct module			*owner;
	int use_count;

	int (*set_trigger_state)(struct iio_trigger *trig, bool state);
	int (*try_reenable)(struct iio_trigger *trig);
};

static inline struct iio_trigger *to_iio_trigger(struct device *d)
{
	return container_of(d, struct iio_trigger, dev);
};

static inline void iio_put_trigger(struct iio_trigger *trig)
{
	put_device(&trig->dev);
	module_put(trig->owner);
};

static inline void iio_get_trigger(struct iio_trigger *trig)
{
	__module_get(trig->owner);
	get_device(&trig->dev);
};

ssize_t iio_trigger_read_name(struct device *dev,
			      struct device_attribute *attr,
			      char *buf);

#define IIO_TRIGGER_NAME_ATTR DEVICE_ATTR(name, S_IRUGO,		\
					  iio_trigger_read_name,	\
					  NULL);

struct iio_trigger *iio_trigger_find_by_name(const char *name, size_t len);

int iio_trigger_register(struct iio_trigger *trig_info);

void iio_trigger_unregister(struct iio_trigger *trig_info);

int iio_trigger_attach_poll_func(struct iio_trigger *trig,
				 struct iio_poll_func *pf);

int iio_trigger_dettach_poll_func(struct iio_trigger *trig,
				  struct iio_poll_func *pf);

void iio_trigger_poll(struct iio_trigger *trig);
void iio_trigger_notify_done(struct iio_trigger *trig);

struct iio_poll_func {
	struct				list_head list;
	void				*private_data;
	void (*poll_func_immediate)(struct iio_dev *indio_dev);
	void (*poll_func_main)(struct iio_dev  *private_data);

};

struct iio_trigger *iio_allocate_trigger(void);

void iio_free_trigger(struct iio_trigger *trig);


#endif /* _IIO_TRIGGER_H_ */
