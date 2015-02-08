


#ifndef __YAFFS_BITMAP_H__
#define __YAFFS_BITMAP_H__

#include "yaffs_guts.h"

void yaffs_VerifyChunkBitId(yaffs_Device *dev, int blk, int chunk);
void yaffs_ClearChunkBits(yaffs_Device *dev, int blk);
void yaffs_ClearChunkBit(yaffs_Device *dev, int blk, int chunk);
void yaffs_SetChunkBit(yaffs_Device *dev, int blk, int chunk);
int yaffs_CheckChunkBit(yaffs_Device *dev, int blk, int chunk);
int yaffs_StillSomeChunkBits(yaffs_Device *dev, int blk);
int yaffs_CountChunkBits(yaffs_Device *dev, int blk);

#endif
