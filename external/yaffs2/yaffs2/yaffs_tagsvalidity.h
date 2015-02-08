
//yaffs_tagsvalidity.h

#ifndef __YAFFS_TAGS_VALIDITY_H__
#define __YAFFS_TAGS_VALIDITY_H__

#include "yaffs_guts.h"

void yaffs_InitialiseTags(yaffs_ExtendedTags * tags);
int yaffs_ValidateTags(yaffs_ExtendedTags * tags);
#endif
