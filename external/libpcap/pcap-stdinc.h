

#define SIZEOF_CHAR 1
#define SIZEOF_SHORT 2
#define SIZEOF_INT 4
#ifndef _MSC_EXTENSIONS
#define SIZEOF_LONG_LONG 8
#endif

#ifdef _WINSOCKAPI_
#undef _WINSOCKAPI_
#endif
#include <winsock2.h>

#include <fcntl.h>

#include "bittypes.h"
#include <time.h>
#include <io.h>

#ifndef __MINGW32__
#include "IP6_misc.h"
#endif

#define caddr_t char*

#define snprintf _snprintf
#define vsnprintf _vsnprintf
#define inline __inline 
