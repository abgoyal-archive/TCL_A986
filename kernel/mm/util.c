
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

#define CREATE_TRACE_POINTS
#include <trace/events/kmem.h>

char *kstrdup(const char *s, gfp_t gfp)
{
	size_t len;
	char *buf;

	if (!s)
		return NULL;

	len = strlen(s) + 1;
	buf = kmalloc_track_caller(len, gfp);
	if (buf)
		memcpy(buf, s, len);
	return buf;
}
EXPORT_SYMBOL(kstrdup);

char *kstrndup(const char *s, size_t max, gfp_t gfp)
{
	size_t len;
	char *buf;

	if (!s)
		return NULL;

	len = strnlen(s, max);
	buf = kmalloc_track_caller(len+1, gfp);
	if (buf) {
		memcpy(buf, s, len);
		buf[len] = '\0';
	}
	return buf;
}
EXPORT_SYMBOL(kstrndup);

void *kmemdup(const void *src, size_t len, gfp_t gfp)
{
	void *p;

	p = kmalloc_track_caller(len, gfp);
	if (p)
		memcpy(p, src, len);
	return p;
}
EXPORT_SYMBOL(kmemdup);

void *memdup_user(const void __user *src, size_t len)
{
	void *p;

	/*
	 * Always use GFP_KERNEL, since copy_from_user() can sleep and
	 * cause pagefault, which makes it pointless to use GFP_NOFS
	 * or GFP_ATOMIC.
	 */
	p = kmalloc_track_caller(len, GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}

	return p;
}
EXPORT_SYMBOL(memdup_user);

void *__krealloc(const void *p, size_t new_size, gfp_t flags)
{
	void *ret;
	size_t ks = 0;

	if (unlikely(!new_size))
		return ZERO_SIZE_PTR;

	if (p)
		ks = ksize(p);

	if (ks >= new_size)
		return (void *)p;

	ret = kmalloc_track_caller(new_size, flags);
	if (ret && p)
		memcpy(ret, p, ks);

	return ret;
}
EXPORT_SYMBOL(__krealloc);

void *krealloc(const void *p, size_t new_size, gfp_t flags)
{
	void *ret;

	if (unlikely(!new_size)) {
		kfree(p);
		return ZERO_SIZE_PTR;
	}

	ret = __krealloc(p, new_size, flags);
	if (ret && p != ret)
		kfree(p);

	return ret;
}
EXPORT_SYMBOL(krealloc);

void kzfree(const void *p)
{
	size_t ks;
	void *mem = (void *)p;

	if (unlikely(ZERO_OR_NULL_PTR(mem)))
		return;
	ks = ksize(mem);
	memset(mem, 0, ks);
	kfree(mem);
}
EXPORT_SYMBOL(kzfree);

int kern_ptr_validate(const void *ptr, unsigned long size)
{
	unsigned long addr = (unsigned long)ptr;
	unsigned long min_addr = PAGE_OFFSET;
	unsigned long align_mask = sizeof(void *) - 1;

	if (unlikely(addr < min_addr))
		goto out;
	if (unlikely(addr > (unsigned long)high_memory - size))
		goto out;
	if (unlikely(addr & align_mask))
		goto out;
	if (unlikely(!kern_addr_valid(addr)))
		goto out;
	if (unlikely(!kern_addr_valid(addr + size - 1)))
		goto out;
	return 1;
out:
	return 0;
}

char *strndup_user(const char __user *s, long n)
{
	char *p;
	long length;

	length = strnlen_user(s, n);

	if (!length)
		return ERR_PTR(-EFAULT);

	if (length > n)
		return ERR_PTR(-EINVAL);

	p = kmalloc(length, GFP_KERNEL);

	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, s, length)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}

	p[length - 1] = '\0';

	return p;
}
EXPORT_SYMBOL(strndup_user);

#if defined(CONFIG_MMU) && !defined(HAVE_ARCH_PICK_MMAP_LAYOUT)
void arch_pick_mmap_layout(struct mm_struct *mm)
{
	mm->mmap_base = TASK_UNMAPPED_BASE;
	mm->get_unmapped_area = arch_get_unmapped_area;
	mm->unmap_area = arch_unmap_area;
}
#endif

int __attribute__((weak)) get_user_pages_fast(unsigned long start,
				int nr_pages, int write, struct page **pages)
{
	struct mm_struct *mm = current->mm;
	int ret;

	down_read(&mm->mmap_sem);
	ret = get_user_pages(current, mm, start, nr_pages,
					write, 0, pages, NULL);
	up_read(&mm->mmap_sem);

	return ret;
}
EXPORT_SYMBOL_GPL(get_user_pages_fast);

/* Tracepoints definitions. */
EXPORT_TRACEPOINT_SYMBOL(kmalloc);
EXPORT_TRACEPOINT_SYMBOL(kmem_cache_alloc);
EXPORT_TRACEPOINT_SYMBOL(kmalloc_node);
EXPORT_TRACEPOINT_SYMBOL(kmem_cache_alloc_node);
EXPORT_TRACEPOINT_SYMBOL(kfree);
EXPORT_TRACEPOINT_SYMBOL(kmem_cache_free);
