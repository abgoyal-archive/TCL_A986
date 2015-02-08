
#ifndef __YAFFS_NAND_H__
#define __YAFFS_NAND_H__
#include "yaffs_guts.h"



int yaffs_ReadChunkWithTagsFromNAND(yaffs_Device * dev, int chunkInNAND,
					   __u8 * buffer,
					   yaffs_ExtendedTags * tags);

int yaffs_WriteChunkWithTagsToNAND(yaffs_Device * dev,
						   int chunkInNAND,
						   const __u8 * buffer,
						   yaffs_ExtendedTags * tags);

int yaffs_MarkBlockBad(yaffs_Device * dev, int blockNo);

int yaffs_QueryInitialBlockState(yaffs_Device * dev,
						 int blockNo,
						 yaffs_BlockState * state,
						 unsigned *sequenceNumber);

int yaffs_EraseBlockInNAND(struct yaffs_DeviceStruct *dev,
				  int blockInNAND);

int yaffs_InitialiseNAND(struct yaffs_DeviceStruct *dev);

#endif

