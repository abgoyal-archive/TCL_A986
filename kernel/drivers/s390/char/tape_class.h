
#ifndef __TAPE_CLASS_H__
#define __TAPE_CLASS_H__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/kobject.h>
#include <linux/kobj_map.h>
#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/kdev_t.h>

#define TAPECLASS_NAME_LEN	32

struct tape_class_device {
	struct cdev		*char_device;
	struct device		*class_device;
	char			device_name[TAPECLASS_NAME_LEN];
	char			mode_name[TAPECLASS_NAME_LEN];
};

struct tape_class_device *register_tape_dev(
	struct device *		device,
	dev_t			dev,
	const struct file_operations *fops,
	char *			device_name,
	char *			node_name
);
void unregister_tape_dev(struct device *device, struct tape_class_device *tcd);

#endif /* __TAPE_CLASS_H__ */
