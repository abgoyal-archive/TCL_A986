

#ifndef _NILFS_BMAP_UNION_H
#define _NILFS_BMAP_UNION_H

#include "bmap.h"
#include "direct.h"
#include "btree.h"

union nilfs_bmap_union {
	struct nilfs_bmap bi_bmap;
	struct nilfs_direct bi_direct;
	struct nilfs_btree bi_btree;
};

#endif	/* _NILFS_BMAP_UNION_H */
