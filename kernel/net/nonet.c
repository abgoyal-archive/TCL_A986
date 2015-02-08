

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>

static int sock_no_open(struct inode *irrelevant, struct file *dontcare)
{
	return -ENXIO;
}

const struct file_operations bad_sock_fops = {
	.owner = THIS_MODULE,
	.open = sock_no_open,
};
