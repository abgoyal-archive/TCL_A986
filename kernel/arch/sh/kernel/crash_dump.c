
#include <linux/errno.h>
#include <linux/crash_dump.h>
#include <linux/io.h>
#include <asm/uaccess.h>

/* Stores the physical address of elf header of crash image. */
unsigned long long elfcorehdr_addr = ELFCORE_ADDR_MAX;

static int __init parse_elfcorehdr(char *arg)
{
	if (!arg)
		return -EINVAL;

	elfcorehdr_addr = memparse(arg, &arg);

	return 0;
}
early_param("elfcorehdr", parse_elfcorehdr);

ssize_t copy_oldmem_page(unsigned long pfn, char *buf,
                               size_t csize, unsigned long offset, int userbuf)
{
	void  *vaddr;

	if (!csize)
		return 0;

	vaddr = ioremap(pfn << PAGE_SHIFT, PAGE_SIZE);

	if (userbuf) {
		if (copy_to_user(buf, (vaddr + offset), csize)) {
			iounmap(vaddr);
			return -EFAULT;
		}
	} else
	memcpy(buf, (vaddr + offset), csize);

	iounmap(vaddr);
	return csize;
}
