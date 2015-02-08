

#ifndef __YAFFS_MTDIF2_H__
#define __YAFFS_MTDIF2_H__

#include "yaffs_guts.h"
int nandmtd2_WriteChunkWithTagsToNAND(yaffs_Device *dev, int chunkInNAND,
				const __u8 *data,
				const yaffs_ExtendedTags *tags);
int nandmtd2_ReadChunkWithTagsFromNAND(yaffs_Device *dev, int chunkInNAND,
				__u8 *data, yaffs_ExtendedTags *tags);
int nandmtd2_MarkNANDBlockBad(struct yaffs_DeviceStruct *dev, int blockNo);
int nandmtd2_QueryNANDBlock(struct yaffs_DeviceStruct *dev, int blockNo,
			yaffs_BlockState *state, __u32 *sequenceNumber);

#endif
