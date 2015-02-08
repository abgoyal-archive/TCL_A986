
#ifndef __YAFFS_MTDIF_H__
#define __YAFFS_MTDIF_H__

#include "yaffs_guts.h"

int nandmtd_WriteChunkToNAND(yaffs_Device * dev, int chunkInNAND,
			     const __u8 * data, const yaffs_Spare * spare);
int nandmtd_ReadChunkFromNAND(yaffs_Device * dev, int chunkInNAND, __u8 * data,
			      yaffs_Spare * spare);
int nandmtd_EraseBlockInNAND(yaffs_Device * dev, int blockNumber);
int nandmtd_InitialiseNAND(yaffs_Device * dev);
#endif
