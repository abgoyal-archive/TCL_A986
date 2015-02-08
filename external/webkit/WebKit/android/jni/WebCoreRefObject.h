

#ifndef WEBCORE_FOUNDATION_h
#define WEBCORE_FOUNDATION_h

#include "SkRefCnt.h"

typedef SkRefCnt WebCoreRefObject;

static inline WebCoreRefObject* Retain(WebCoreRefObject* obj)
{
    if (obj) 
        obj->ref();
    return obj;
}

static inline void Release(WebCoreRefObject* obj)
{
    if (obj) 
        obj->unref();
}

#endif // WEBCORE_FOUNDATION_h
