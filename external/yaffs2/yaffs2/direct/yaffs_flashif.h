
// This provides a rma disk under yaffs.
// NB this is not intended for NAND emulation.
// Use this with dev->useNANDECC enabled, then ECC overheads are not required.

#ifndef __YAFFS_FLASH_H__
#define __YAFFS_FLASH_H__


#include "yaffs_guts.h"
int yflash_EraseBlockInNAND(yaffs_Device *dev, int blockNumber);
int yflash_WriteChunkToNAND(yaffs_Device *dev,int chunkInNAND,const __u8 *data, const yaffs_Spare *spare);
int yflash_WriteChunkWithTagsToNAND(yaffs_Device *dev,int chunkInNAND,const __u8 *data, yaffs_ExtendedTags *tags);
int yflash_ReadChunkFromNAND(yaffs_Device *dev,int chunkInNAND, __u8 *data, yaffs_Spare *spare);
int yflash_ReadChunkWithTagsFromNAND(yaffs_Device *dev,int chunkInNAND, __u8 *data, yaffs_ExtendedTags *tags);
int yflash_EraseBlockInNAND(yaffs_Device *dev, int blockNumber);
int yflash_InitialiseNAND(yaffs_Device *dev);
int yflash_MarkNANDBlockBad(struct yaffs_DeviceStruct *dev, int blockNo);
int yflash_QueryNANDBlock(struct yaffs_DeviceStruct *dev, int blockNo, yaffs_BlockState *state, int *sequenceNumber);

#endif
