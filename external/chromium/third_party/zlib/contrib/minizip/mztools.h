
#ifndef _zip_tools_H
#define _zip_tools_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(USE_SYSTEM_ZLIB)
#include <zlib.h>
#else
#include "third_party/zlib/zlib.h"
#endif

#include "unzip.h"

extern int ZEXPORT unzRepair(const char* file, 
                             const char* fileOut, 
                             const char* fileOutTmp, 
                             uLong* nRecovered,
                             uLong* bytesRecovered);

#endif
