


#ifndef __UMAPFILE_H__
#define __UMAPFILE_H__

#include "unicode/udata.h"

UBool   uprv_mapFile(UDataMemory *pdm, const char *path);
void    uprv_unmapFile(UDataMemory *pData);

#endif
