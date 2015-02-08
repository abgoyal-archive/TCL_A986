

#ifndef __YAFFS_MTDIF1_H__
#define __YAFFS_MTDIF1_H__

int nandmtd1_WriteChunkWithTagsToNAND(yaffs_Device *dev, int chunkInNAND,
	const __u8 *data, const yaffs_ExtendedTags *tags);

int nandmtd1_ReadChunkWithTagsFromNAND(yaffs_Device *dev, int chunkInNAND,
	__u8 *data, yaffs_ExtendedTags *tags);

int nandmtd1_MarkNANDBlockBad(struct yaffs_DeviceStruct *dev, int blockNo);

int nandmtd1_QueryNANDBlock(struct yaffs_DeviceStruct *dev, int blockNo,
	yaffs_BlockState *state, __u32 *sequenceNumber);

#endif
