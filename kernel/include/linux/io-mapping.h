

#ifndef _LINUX_IO_MAPPING_H
#define _LINUX_IO_MAPPING_H

#include <linux/types.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/iomap.h>


#ifdef CONFIG_HAVE_ATOMIC_IOMAP

struct io_mapping {
	resource_size_t base;
	unsigned long size;
	pgprot_t prot;
};


static inline struct io_mapping *
io_mapping_create_wc(resource_size_t base, unsigned long size)
{
	struct io_mapping *iomap;
	pgprot_t prot;

	iomap = kmalloc(sizeof(*iomap), GFP_KERNEL);
	if (!iomap)
		goto out_err;

	if (iomap_create_wc(base, size, &prot))
		goto out_free;

	iomap->base = base;
	iomap->size = size;
	iomap->prot = prot;
	return iomap;

out_free:
	kfree(iomap);
out_err:
	return NULL;
}

static inline void
io_mapping_free(struct io_mapping *mapping)
{
	iomap_free(mapping->base, mapping->size);
	kfree(mapping);
}

/* Atomic map/unmap */
static inline void *
io_mapping_map_atomic_wc(struct io_mapping *mapping, unsigned long offset)
{
	resource_size_t phys_addr;
	unsigned long pfn;

	BUG_ON(offset >= mapping->size);
	phys_addr = mapping->base + offset;
	pfn = (unsigned long) (phys_addr >> PAGE_SHIFT);
	return iomap_atomic_prot_pfn(pfn, KM_USER0, mapping->prot);
}

static inline void
io_mapping_unmap_atomic(void *vaddr)
{
	iounmap_atomic(vaddr, KM_USER0);
}

static inline void *
io_mapping_map_wc(struct io_mapping *mapping, unsigned long offset)
{
	resource_size_t phys_addr;

	BUG_ON(offset >= mapping->size);
	phys_addr = mapping->base + offset;

	return ioremap_wc(phys_addr, PAGE_SIZE);
}

static inline void
io_mapping_unmap(void *vaddr)
{
	iounmap(vaddr);
}

#else

/* this struct isn't actually defined anywhere */
struct io_mapping;

/* Create the io_mapping object*/
static inline struct io_mapping *
io_mapping_create_wc(resource_size_t base, unsigned long size)
{
	return (struct io_mapping *) ioremap_wc(base, size);
}

static inline void
io_mapping_free(struct io_mapping *mapping)
{
	iounmap(mapping);
}

/* Atomic map/unmap */
static inline void *
io_mapping_map_atomic_wc(struct io_mapping *mapping, unsigned long offset)
{
	return ((char *) mapping) + offset;
}

static inline void
io_mapping_unmap_atomic(void *vaddr)
{
}

/* Non-atomic map/unmap */
static inline void *
io_mapping_map_wc(struct io_mapping *mapping, unsigned long offset)
{
	return ((char *) mapping) + offset;
}

static inline void
io_mapping_unmap(void *vaddr)
{
}

#endif /* HAVE_ATOMIC_IOMAP */

#endif /* _LINUX_IO_MAPPING_H */
