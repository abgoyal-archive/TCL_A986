

#ifndef JSStringRefCF_h
#define JSStringRefCF_h

#include "JSBase.h"
#include <CoreFoundation/CoreFoundation.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CFString convenience methods */

JS_EXPORT JSStringRef JSStringCreateWithCFString(CFStringRef string);
JS_EXPORT CFStringRef JSStringCopyCFString(CFAllocatorRef alloc, JSStringRef string);

#ifdef __cplusplus
}
#endif

#endif /* JSStringRefCF_h */
