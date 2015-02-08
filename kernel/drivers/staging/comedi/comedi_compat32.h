

#ifndef _COMEDI_COMPAT32_H
#define _COMEDI_COMPAT32_H

#include <linux/compat.h>
#include <linux/fs.h>

#ifdef CONFIG_COMPAT

extern long comedi_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg);

#else /* CONFIG_COMPAT */

#define comedi_compat_ioctl 0	/* NULL */

#endif /* CONFIG_COMPAT */

#endif /* _COMEDI_COMPAT32_H */
