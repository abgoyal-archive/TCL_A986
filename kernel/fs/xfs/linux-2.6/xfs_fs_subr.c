
#include "xfs.h"
#include "xfs_vnodeops.h"
#include "xfs_bmap_btree.h"
#include "xfs_inode.h"
#include "xfs_trace.h"

int  fs_noerr(void) { return 0; }
int  fs_nosys(void) { return ENOSYS; }
void fs_noval(void) { return; }

void
xfs_tosspages(
	xfs_inode_t	*ip,
	xfs_off_t	first,
	xfs_off_t	last,
	int		fiopt)
{
	struct address_space *mapping = VFS_I(ip)->i_mapping;

	if (mapping->nrpages)
		truncate_inode_pages(mapping, first);
}

int
xfs_flushinval_pages(
	xfs_inode_t	*ip,
	xfs_off_t	first,
	xfs_off_t	last,
	int		fiopt)
{
	struct address_space *mapping = VFS_I(ip)->i_mapping;
	int		ret = 0;

	trace_xfs_pagecache_inval(ip, first, last);

	if (mapping->nrpages) {
		xfs_iflags_clear(ip, XFS_ITRUNCATED);
		ret = filemap_write_and_wait(mapping);
		if (!ret)
			truncate_inode_pages(mapping, first);
	}
	return -ret;
}

int
xfs_flush_pages(
	xfs_inode_t	*ip,
	xfs_off_t	first,
	xfs_off_t	last,
	uint64_t	flags,
	int		fiopt)
{
	struct address_space *mapping = VFS_I(ip)->i_mapping;
	int		ret = 0;
	int		ret2;

	if (mapping_tagged(mapping, PAGECACHE_TAG_DIRTY)) {
		xfs_iflags_clear(ip, XFS_ITRUNCATED);
		ret = -filemap_fdatawrite(mapping);
	}
	if (flags & XBF_ASYNC)
		return ret;
	ret2 = xfs_wait_on_pages(ip, first, last);
	if (!ret)
		ret = ret2;
	return ret;
}

int
xfs_wait_on_pages(
	xfs_inode_t	*ip,
	xfs_off_t	first,
	xfs_off_t	last)
{
	struct address_space *mapping = VFS_I(ip)->i_mapping;

	if (mapping_tagged(mapping, PAGECACHE_TAG_WRITEBACK))
		return -filemap_fdatawait(mapping);
	return 0;
}
