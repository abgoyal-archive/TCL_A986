
#ifndef __YAFFS_NANDEMUL2K_H__
#define __YAFFS_NANDEMUL2K_H__

#include "yaffs_guts.h"

int nandemul2k_WriteChunkWithTagsToNAND(struct yaffs_DeviceStruct *dev,
					int chunkInNAND, const __u8 * data,
					yaffs_ExtendedTags * tags);
int nandemul2k_ReadChunkWithTagsFromNAND(struct yaffs_DeviceStruct *dev,
					 int chunkInNAND, __u8 * data,
					 yaffs_ExtendedTags * tags);
int nandemul2k_MarkNANDBlockBad(struct yaffs_DeviceStruct *dev, int blockNo);
int nandemul2k_QueryNANDBlock(struct yaffs_DeviceStruct *dev, int blockNo,
			      yaffs_BlockState * state, int *sequenceNumber);
int nandemul2k_EraseBlockInNAND(struct yaffs_DeviceStruct *dev,
				int blockInNAND);
int nandemul2k_InitialiseNAND(struct yaffs_DeviceStruct *dev);
int nandemul2k_GetBytesPerChunk(void);
int nandemul2k_GetChunksPerBlock(void);
int nandemul2k_GetNumberOfBlocks(void);

#endif
