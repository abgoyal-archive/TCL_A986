

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>

#include <asm/uaccess.h>

static void *kdump_buf_page;

/* Stores the physical address of elf header of crash image. */
unsigned long long elfcorehdr_addr = ELFCORE_ADDR_MAX;

static inline bool is_crashed_pfn_valid(unsigned long pfn)
{
#ifndef CONFIG_X86_PAE
	/*
	 * non-PAE kdump kernel executed from a PAE one will crop high pte
	 * bits and poke unwanted space counting again from address 0, we
	 * don't want that. pte must fit into unsigned long. In fact the
	 * test checks high 12 bits for being zero (pfn will be shifted left
	 * by PAGE_SHIFT).
	 */
	return pte_pfn(pfn_pte(pfn, __pgprot(0))) == pfn;
#else
	return true;
#endif
}

ssize_t copy_oldmem_page(unsigned long pfn, char *buf,
                               size_t csize, unsigned long offset, int userbuf)
{
	void  *vaddr;

	if (!csize)
		return 0;

	if (!is_crashed_pfn_valid(pfn))
		return -EFAULT;

	vaddr = kmap_atomic_pfn(pfn, KM_PTE0);

	if (!userbuf) {
		memcpy(buf, (vaddr + offset), csize);
		kunmap_atomic(vaddr, KM_PTE0);
	} else {
		if (!kdump_buf_page) {
			printk(KERN_WARNING "Kdump: Kdump buffer page not"
				" allocated\n");
			kunmap_atomic(vaddr, KM_PTE0);
			return -EFAULT;
		}
		copy_page(kdump_buf_page, vaddr);
		kunmap_atomic(vaddr, KM_PTE0);
		if (copy_to_user(buf, (kdump_buf_page + offset), csize))
			return -EFAULT;
	}

	return csize;
}

static int __init kdump_buf_page_init(void)
{
	int ret = 0;

	kdump_buf_page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!kdump_buf_page) {
		printk(KERN_WARNING "Kdump: Failed to allocate kdump buffer"
			 " page\n");
		ret = -ENOMEM;
	}

	return ret;
}
arch_initcall(kdump_buf_page_init);
