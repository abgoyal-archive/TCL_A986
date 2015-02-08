

#ifndef __YAFFS_GETBLOCKINFO_H__
#define __YAFFS_GETBLOCKINFO_H__

#include "yaffs_guts.h"
#include "yaffs_trace.h"

/* Function to manipulate block info */
static Y_INLINE yaffs_BlockInfo *yaffs_GetBlockInfo(yaffs_Device * dev, int blk)
{
	if (blk < dev->internalStartBlock || blk > dev->internalEndBlock) {
		T(YAFFS_TRACE_ERROR,
		  (TSTR
		   ("**>> yaffs: getBlockInfo block %d is not valid" TENDSTR),
		   blk));
		YBUG();
	}
	return &dev->blockInfo[blk - dev->internalStartBlock];
}

#endif
