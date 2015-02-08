

#include <linux/oprofile.h>
#include <linux/init.h>
#include <linux/errno.h>


extern void s390_backtrace(struct pt_regs * const regs, unsigned int depth);

int __init oprofile_arch_init(struct oprofile_operations* ops)
{
	ops->backtrace = s390_backtrace;
	return -ENODEV;
}

void oprofile_arch_exit(void)
{
}
