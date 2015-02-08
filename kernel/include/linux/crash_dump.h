
#ifndef LINUX_CRASH_DUMP_H
#define LINUX_CRASH_DUMP_H

#ifdef CONFIG_CRASH_DUMP
#include <linux/kexec.h>
#include <linux/device.h>
#include <linux/proc_fs.h>

#define ELFCORE_ADDR_MAX	(-1ULL)
#define ELFCORE_ADDR_ERR	(-2ULL)

extern unsigned long long elfcorehdr_addr;

extern ssize_t copy_oldmem_page(unsigned long, char *, size_t,
						unsigned long, int);

#ifndef vmcore_elf_check_arch_cross
#define vmcore_elf_check_arch_cross(x) 0
#endif

#define vmcore_elf_check_arch(x) (elf_check_arch(x) || vmcore_elf_check_arch_cross(x))


static inline int is_kdump_kernel(void)
{
	return (elfcorehdr_addr != ELFCORE_ADDR_MAX) ? 1 : 0;
}


static inline int is_vmcore_usable(void)
{
	return is_kdump_kernel() && elfcorehdr_addr != ELFCORE_ADDR_ERR ? 1 : 0;
}


static inline void vmcore_unusable(void)
{
	if (is_kdump_kernel())
		elfcorehdr_addr = ELFCORE_ADDR_ERR;
}
#else /* !CONFIG_CRASH_DUMP */
static inline int is_kdump_kernel(void) { return 0; }
#endif /* CONFIG_CRASH_DUMP */

extern unsigned long saved_max_pfn;
#endif /* LINUX_CRASHDUMP_H */
