

#ifndef _LINUX_NTFS_MALLOC_H
#define _LINUX_NTFS_MALLOC_H

#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/highmem.h>

static inline void *__ntfs_malloc(unsigned long size, gfp_t gfp_mask)
{
	if (likely(size <= PAGE_SIZE)) {
		BUG_ON(!size);
		/* kmalloc() has per-CPU caches so is faster for now. */
		return kmalloc(PAGE_SIZE, gfp_mask & ~__GFP_HIGHMEM);
		/* return (void *)__get_free_page(gfp_mask); */
	}
	if (likely((size >> PAGE_SHIFT) < totalram_pages))
		return __vmalloc(size, gfp_mask, PAGE_KERNEL);
	return NULL;
}

static inline void *ntfs_malloc_nofs(unsigned long size)
{
	return __ntfs_malloc(size, GFP_NOFS | __GFP_HIGHMEM);
}

static inline void *ntfs_malloc_nofs_nofail(unsigned long size)
{
	return __ntfs_malloc(size, GFP_NOFS | __GFP_HIGHMEM | __GFP_NOFAIL);
}

static inline void ntfs_free(void *addr)
{
	if (!is_vmalloc_addr(addr)) {
		kfree(addr);
		/* free_page((unsigned long)addr); */
		return;
	}
	vfree(addr);
}

#endif /* _LINUX_NTFS_MALLOC_H */
