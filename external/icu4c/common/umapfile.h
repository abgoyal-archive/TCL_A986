

#ifndef __UMAPFILE_H__
#define __UMAPFILE_H__

#include "unicode/putil.h"
#include "unicode/udata.h"

UBool   uprv_mapFile(UDataMemory *pdm, const char *path);
void    uprv_unmapFile(UDataMemory *pData);

/* MAP_NONE: no memory mapping, no file access at all */
#define MAP_NONE        0
#define MAP_WIN32       1
#define MAP_POSIX       2
#define MAP_STDIO       3
#define MAP_390DLL      4

#if UCONFIG_NO_FILE_IO
#   define MAP_IMPLEMENTATION MAP_NONE
#elif defined(U_WINDOWS)
#   define MAP_IMPLEMENTATION MAP_WIN32
#elif U_HAVE_MMAP || defined(OS390)
#   if defined(OS390) && defined (OS390_STUBDATA)
        /*   No memory mapping for 390 batch mode.  Fake it using dll loading.  */
#       define MAP_IMPLEMENTATION MAP_390DLL
#   else
#       define MAP_IMPLEMENTATION MAP_POSIX
#   endif
#else /* unknown platform, no memory map implementation: use stdio.h and uprv_malloc() instead */
#   define MAP_IMPLEMENTATION MAP_STDIO
#endif

#endif
