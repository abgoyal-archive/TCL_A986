
#ifndef __XFS_VNODE_H__
#define __XFS_VNODE_H__

#include "xfs_fs.h"

struct file;
struct xfs_inode;
struct xfs_iomap;
struct attrlist_cursor_kern;

#define	VN_INACTIVE_CACHE	0
#define	VN_INACTIVE_NOCACHE	1

#define IO_ISDIRECT	0x00004		/* bypass page cache */
#define IO_INVIS	0x00020		/* don't update inode timestamps */

#define XFS_IO_FLAGS \
	{ IO_ISDIRECT,	"DIRECT" }, \
	{ IO_INVIS,	"INVIS"}

#define FI_NONE			0	/* none */
#define FI_REMAPF		1	/* Do a remapf prior to the operation */
#define FI_REMAPF_LOCKED	2	/* Do a remapf prior to the operation.
					   Prevent VM access to the pages until
					   the operation completes. */

#define VN_MAPPED(vp)	mapping_mapped(vp->i_mapping)
#define VN_CACHED(vp)	(vp->i_mapping->nrpages)
#define VN_DIRTY(vp)	mapping_tagged(vp->i_mapping, \
					PAGECACHE_TAG_DIRTY)


#endif	/* __XFS_VNODE_H__ */
