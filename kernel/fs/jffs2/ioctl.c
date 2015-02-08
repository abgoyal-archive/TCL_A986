

#include <linux/fs.h>
#include "nodelist.h"

long jffs2_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Later, this will provide for lsattr.jffs2 and chattr.jffs2, which
	   will include compression support etc. */
	return -ENOTTY;
}

