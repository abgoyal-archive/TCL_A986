

#ifndef __YAFFS_CHECKPTRW_H__
#define __YAFFS_CHECKPTRW_H__

#include "yaffs_guts.h"

int yaffs2_CheckpointOpen(yaffs_Device *dev, int forWriting);

int yaffs2_CheckpointWrite(yaffs_Device *dev, const void *data, int nBytes);

int yaffs2_CheckpointRead(yaffs_Device *dev, void *data, int nBytes);

int yaffs2_GetCheckpointSum(yaffs_Device *dev, __u32 *sum);

int yaffs2_CheckpointClose(yaffs_Device *dev);

int yaffs2_CheckpointInvalidateStream(yaffs_Device *dev);


#endif
