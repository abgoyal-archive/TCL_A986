
#include <linux/kernel.h>
#include <linux/oprofile.h>
#include <linux/init.h>
#include <linux/errno.h>

int __init oprofile_arch_init(struct oprofile_operations *ops)
{
	return -ENODEV;
}

void oprofile_arch_exit(void)
{
}

