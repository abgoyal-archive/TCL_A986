

#ifndef _NILFS_PAGE_H
#define _NILFS_PAGE_H

#include <linux/buffer_head.h>
#include "nilfs.h"

enum {
	BH_NILFS_Allocated = BH_PrivateStart,
	BH_NILFS_Node,
	BH_NILFS_Volatile,
};

BUFFER_FNS(NILFS_Allocated, nilfs_allocated)	/* nilfs private buffers */
BUFFER_FNS(NILFS_Node, nilfs_node)		/* nilfs node buffers */
BUFFER_FNS(NILFS_Volatile, nilfs_volatile)


void nilfs_mark_buffer_dirty(struct buffer_head *bh);
int __nilfs_clear_page_dirty(struct page *);

struct buffer_head *nilfs_grab_buffer(struct inode *, struct address_space *,
				      unsigned long, unsigned long);
void nilfs_forget_buffer(struct buffer_head *);
void nilfs_copy_buffer(struct buffer_head *, struct buffer_head *);
int nilfs_page_buffers_clean(struct page *);
void nilfs_page_bug(struct page *);
struct page *nilfs_alloc_private_page(struct block_device *, int,
				      unsigned long);
void nilfs_free_private_page(struct page *);

int nilfs_copy_dirty_pages(struct address_space *, struct address_space *);
void nilfs_copy_back_pages(struct address_space *, struct address_space *);
void nilfs_clear_dirty_pages(struct address_space *);
unsigned nilfs_page_count_clean_buffers(struct page *, unsigned, unsigned);

#define NILFS_PAGE_BUG(page, m, a...) \
	do { nilfs_page_bug(page); BUG(); } while (0)

static inline struct buffer_head *
nilfs_page_get_nth_block(struct page *page, unsigned int count)
{
	struct buffer_head *bh = page_buffers(page);

	while (count-- > 0)
		bh = bh->b_this_page;
	get_bh(bh);
	return bh;
}

#endif /* _NILFS_PAGE_H */
