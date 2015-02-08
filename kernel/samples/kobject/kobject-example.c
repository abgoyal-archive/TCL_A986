
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>


static int foo;
static int baz;
static int bar;

static ssize_t foo_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", foo);
}

static ssize_t foo_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%du", &foo);
	return count;
}

static struct kobj_attribute foo_attribute =
	__ATTR(foo, 0666, foo_show, foo_store);

static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int var;

	if (strcmp(attr->attr.name, "baz") == 0)
		var = baz;
	else
		var = bar;
	return sprintf(buf, "%d\n", var);
}

static ssize_t b_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	int var;

	sscanf(buf, "%du", &var);
	if (strcmp(attr->attr.name, "baz") == 0)
		baz = var;
	else
		bar = var;
	return count;
}

static struct kobj_attribute baz_attribute =
	__ATTR(baz, 0666, b_show, b_store);
static struct kobj_attribute bar_attribute =
	__ATTR(bar, 0666, b_show, b_store);


static struct attribute *attrs[] = {
	&foo_attribute.attr,
	&baz_attribute.attr,
	&bar_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *example_kobj;

static int __init example_init(void)
{
	int retval;

	/*
	 * Create a simple kobject with the name of "kobject_example",
	 * located under /sys/kernel/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */
	example_kobj = kobject_create_and_add("kobject_example", kernel_kobj);
	if (!example_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(example_kobj, &attr_group);
	if (retval)
		kobject_put(example_kobj);

	return retval;
}

static void __exit example_exit(void)
{
	kobject_put(example_kobj);
}

module_init(example_init);
module_exit(example_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Greg Kroah-Hartman <greg@kroah.com>");
