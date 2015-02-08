

#ifndef _LINUX_NTFS_AOPS_H
#define _LINUX_NTFS_AOPS_H

#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/fs.h>

#include "inode.h"

static inline void ntfs_unmap_page(struct page *page)
{
	kunmap(page);
	page_cache_release(page);
}

static inline struct page *ntfs_map_page(struct address_space *mapping,
		unsigned long index)
{
	struct page *page = read_mapping_page(mapping, index, NULL);

	if (!IS_ERR(page)) {
		kmap(page);
		if (!PageError(page))
			return page;
		ntfs_unmap_page(page);
		return ERR_PTR(-EIO);
	}
	return page;
}

#ifdef NTFS_RW

extern void mark_ntfs_record_dirty(struct page *page, const unsigned int ofs);

#endif /* NTFS_RW */

#endif /* _LINUX_NTFS_AOPS_H */
