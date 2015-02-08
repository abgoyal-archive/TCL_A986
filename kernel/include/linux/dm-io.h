

#ifndef _LINUX_DM_IO_H
#define _LINUX_DM_IO_H

#ifdef __KERNEL__

#include <linux/types.h>

struct dm_io_region {
	struct block_device *bdev;
	sector_t sector;
	sector_t count;		/* If this is zero the region is ignored. */
};

struct page_list {
	struct page_list *next;
	struct page *page;
};

typedef void (*io_notify_fn)(unsigned long error, void *context);

enum dm_io_mem_type {
	DM_IO_PAGE_LIST,/* Page list */
	DM_IO_BVEC,	/* Bio vector */
	DM_IO_VMA,	/* Virtual memory area */
	DM_IO_KMEM,	/* Kernel memory */
};

struct dm_io_memory {
	enum dm_io_mem_type type;

	unsigned offset;

	union {
		struct page_list *pl;
		struct bio_vec *bvec;
		void *vma;
		void *addr;
	} ptr;
};

struct dm_io_notify {
	io_notify_fn fn;	/* Callback for asynchronous requests */
	void *context;		/* Passed to callback */
};

struct dm_io_client;
struct dm_io_request {
	int bi_rw;			/* READ|WRITE - not READA */
	struct dm_io_memory mem;	/* Memory to use for io */
	struct dm_io_notify notify;	/* Synchronous if notify.fn is NULL */
	struct dm_io_client *client;	/* Client memory handler */
};

struct dm_io_client *dm_io_client_create(unsigned num_pages);
int dm_io_client_resize(unsigned num_pages, struct dm_io_client *client);
void dm_io_client_destroy(struct dm_io_client *client);

int dm_io(struct dm_io_request *io_req, unsigned num_regions,
	  struct dm_io_region *region, unsigned long *sync_error_bits);

#endif	/* __KERNEL__ */
#endif	/* _LINUX_DM_IO_H */
