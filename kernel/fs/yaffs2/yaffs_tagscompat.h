

#ifndef __YAFFS_TAGSCOMPAT_H__
#define __YAFFS_TAGSCOMPAT_H__

#include "yaffs_guts.h"
int yaffs_TagsCompatabilityWriteChunkWithTagsToNAND(yaffs_Device *dev,
						int chunkInNAND,
						const __u8 *data,
						const yaffs_ExtendedTags *tags);
int yaffs_TagsCompatabilityReadChunkWithTagsFromNAND(yaffs_Device *dev,
						int chunkInNAND,
						__u8 *data,
						yaffs_ExtendedTags *tags);
int yaffs_TagsCompatabilityMarkNANDBlockBad(struct yaffs_DeviceStruct *dev,
					    int blockNo);
int yaffs_TagsCompatabilityQueryNANDBlock(struct yaffs_DeviceStruct *dev,
					  int blockNo,
					  yaffs_BlockState *state,
					  __u32 *sequenceNumber);

void yaffs_CalcTagsECC(yaffs_Tags *tags);
int yaffs_CheckECCOnTags(yaffs_Tags *tags);
int yaffs_CountBits(__u8 byte);

#endif
