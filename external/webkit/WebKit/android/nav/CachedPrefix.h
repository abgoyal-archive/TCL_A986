

#ifndef CachedPrefix_H
#define CachedPrefix_H

#ifndef LOG_TAG
#define LOG_TAG "navcache"
#endif

#include "config.h"
#include "CachedDebug.h"

#ifndef _LIBS_CUTILS_LOG_H
    #ifdef LOG
    #undef LOG
    #endif

    #include <utils/Log.h>
#endif

#define OFFSETOF(type, field) ((char*)&(((type*)1)->field) - (char*)1) // avoids gnu warning

#endif
