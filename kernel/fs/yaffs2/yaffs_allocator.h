

#ifndef __YAFFS_ALLOCATOR_H__
#define __YAFFS_ALLOCATOR_H__

#include "yaffs_guts.h"

void yaffs_InitialiseRawTnodesAndObjects(yaffs_Device *dev);
void yaffs_DeinitialiseRawTnodesAndObjects(yaffs_Device *dev);

yaffs_Tnode *yaffs_AllocateRawTnode(yaffs_Device *dev);
void yaffs_FreeRawTnode(yaffs_Device *dev, yaffs_Tnode *tn);

yaffs_Object *yaffs_AllocateRawObject(yaffs_Device *dev);
void yaffs_FreeRawObject(yaffs_Device *dev, yaffs_Object *obj);

#endif
