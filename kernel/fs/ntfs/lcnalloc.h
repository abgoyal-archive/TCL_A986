

#ifndef _LINUX_NTFS_LCNALLOC_H
#define _LINUX_NTFS_LCNALLOC_H

#ifdef NTFS_RW

#include <linux/fs.h>

#include "attrib.h"
#include "types.h"
#include "inode.h"
#include "runlist.h"
#include "volume.h"

typedef enum {
	FIRST_ZONE	= 0,	/* For sanity checking. */
	MFT_ZONE	= 0,	/* Allocate from $MFT zone. */
	DATA_ZONE	= 1,	/* Allocate from $DATA zone. */
	LAST_ZONE	= 1,	/* For sanity checking. */
} NTFS_CLUSTER_ALLOCATION_ZONES;

extern runlist_element *ntfs_cluster_alloc(ntfs_volume *vol,
		const VCN start_vcn, const s64 count, const LCN start_lcn,
		const NTFS_CLUSTER_ALLOCATION_ZONES zone,
		const bool is_extension);

extern s64 __ntfs_cluster_free(ntfs_inode *ni, const VCN start_vcn,
		s64 count, ntfs_attr_search_ctx *ctx, const bool is_rollback);

static inline s64 ntfs_cluster_free(ntfs_inode *ni, const VCN start_vcn,
		s64 count, ntfs_attr_search_ctx *ctx)
{
	return __ntfs_cluster_free(ni, start_vcn, count, ctx, false);
}

extern int ntfs_cluster_free_from_rl_nolock(ntfs_volume *vol,
		const runlist_element *rl);

static inline int ntfs_cluster_free_from_rl(ntfs_volume *vol,
		const runlist_element *rl)
{
	int ret;

	down_write(&vol->lcnbmp_lock);
	ret = ntfs_cluster_free_from_rl_nolock(vol, rl);
	up_write(&vol->lcnbmp_lock);
	return ret;
}

#endif /* NTFS_RW */

#endif /* defined _LINUX_NTFS_LCNALLOC_H */
