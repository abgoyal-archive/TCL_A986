


#define __NO_VERSION__
#include "comedidev.h"
#include "comedi_fops.h"
#include <linux/proc_fs.h>
#include <linux/string.h>

#ifdef CONFIG_PROC_FS
static int comedi_read(char *buf, char **start, off_t offset, int len,
		       int *eof, void *data)
{
	int i;
	int devices_q = 0;
	int l = 0;
	struct comedi_driver *driv;

	l += sprintf(buf + l,
		     "comedi version " COMEDI_RELEASE "\n"
		     "format string: %s\n",
		     "\"%2d: %-20s %-20s %4d\", i, "
		     "driver_name, board_name, n_subdevices");

	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		struct comedi_device_file_info *dev_file_info =
		    comedi_get_device_file_info(i);
		struct comedi_device *dev;

		if (dev_file_info == NULL)
			continue;
		dev = dev_file_info->device;

		if (dev->attached) {
			devices_q = 1;
			l += sprintf(buf + l, "%2d: %-20s %-20s %4d\n",
				     i,
				     dev->driver->driver_name,
				     dev->board_name, dev->n_subdevices);
		}
	}
	if (!devices_q)
		l += sprintf(buf + l, "no devices\n");

	for (driv = comedi_drivers; driv; driv = driv->next) {
		l += sprintf(buf + l, "%s:\n", driv->driver_name);
		for (i = 0; i < driv->num_names; i++) {
			l += sprintf(buf + l, " %s\n",
				     *(char **)((char *)driv->board_name +
						i * driv->offset));
		}
		if (!driv->num_names)
			l += sprintf(buf + l, " %s\n", driv->driver_name);
	}

	return l;
}

void comedi_proc_init(void)
{
	struct proc_dir_entry *comedi_proc;

	comedi_proc = create_proc_entry("comedi", S_IFREG | S_IRUGO, NULL);
	if (comedi_proc)
		comedi_proc->read_proc = comedi_read;
}

void comedi_proc_cleanup(void)
{
	remove_proc_entry("comedi", NULL);
}
#endif
