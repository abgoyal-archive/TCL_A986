
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/kdev_t.h>
#include <linux/syscalls.h>

static int __init default_rootfs(void)
{
	int err;

	err = sys_mkdir("/dev", 0755);
	if (err < 0)
		goto out;

	err = sys_mknod((const char __user *) "/dev/console",
			S_IFCHR | S_IRUSR | S_IWUSR,
			new_encode_dev(MKDEV(5, 1)));
	if (err < 0)
		goto out;

	err = sys_mkdir("/root", 0700);
	if (err < 0)
		goto out;

	return 0;

out:
	printk(KERN_WARNING "Failed to create a rootfs\n");
	return err;
}
rootfs_initcall(default_rootfs);
