

#ifndef _V4L2_DEVICE_H
#define _V4L2_DEVICE_H

#include <media/v4l2-subdev.h>


#define V4L2_DEVICE_NAME_SIZE (20 + 16)

struct v4l2_device {
	/* dev->driver_data points to this struct.
	   Note: dev might be NULL if there is no parent device
	   as is the case with e.g. ISA devices. */
	struct device *dev;
	/* used to keep track of the registered subdevs */
	struct list_head subdevs;
	/* lock this struct; can be used by the driver as well if this
	   struct is embedded into a larger struct. */
	spinlock_t lock;
	/* unique device name, by default the driver name + bus ID */
	char name[V4L2_DEVICE_NAME_SIZE];
	/* notify callback called by some sub-devices. */
	void (*notify)(struct v4l2_subdev *sd,
			unsigned int notification, void *arg);
};

int __must_check v4l2_device_register(struct device *dev, struct v4l2_device *v4l2_dev);

int v4l2_device_set_name(struct v4l2_device *v4l2_dev, const char *basename,
						atomic_t *instance);

void v4l2_device_disconnect(struct v4l2_device *v4l2_dev);

/* Unregister all sub-devices and any other resources related to v4l2_dev. */
void v4l2_device_unregister(struct v4l2_device *v4l2_dev);

int __must_check v4l2_device_register_subdev(struct v4l2_device *v4l2_dev,
						struct v4l2_subdev *sd);
void v4l2_device_unregister_subdev(struct v4l2_subdev *sd);

/* Iterate over all subdevs. */
#define v4l2_device_for_each_subdev(sd, v4l2_dev)			\
	list_for_each_entry(sd, &(v4l2_dev)->subdevs, list)

#define __v4l2_device_call_subdevs(v4l2_dev, cond, o, f, args...) 	\
	do { 								\
		struct v4l2_subdev *sd; 				\
									\
		list_for_each_entry(sd, &(v4l2_dev)->subdevs, list)   	\
			if ((cond) && sd->ops->o && sd->ops->o->f) 	\
				sd->ops->o->f(sd , ##args); 		\
	} while (0)

#define __v4l2_device_call_subdevs_until_err(v4l2_dev, cond, o, f, args...) \
({ 									\
	struct v4l2_subdev *sd; 					\
	long err = 0; 							\
									\
	list_for_each_entry(sd, &(v4l2_dev)->subdevs, list) { 		\
		if ((cond) && sd->ops->o && sd->ops->o->f) 		\
			err = sd->ops->o->f(sd , ##args); 		\
		if (err && err != -ENOIOCTLCMD)				\
			break; 						\
	} 								\
	(err == -ENOIOCTLCMD) ? 0 : err; 				\
})

#define v4l2_device_call_all(v4l2_dev, grpid, o, f, args...) 		\
	__v4l2_device_call_subdevs(v4l2_dev, 				\
			!(grpid) || sd->grp_id == (grpid), o, f , ##args)

#define v4l2_device_call_until_err(v4l2_dev, grpid, o, f, args...) 	\
	__v4l2_device_call_subdevs_until_err(v4l2_dev,			\
		       !(grpid) || sd->grp_id == (grpid), o, f , ##args)

#endif
