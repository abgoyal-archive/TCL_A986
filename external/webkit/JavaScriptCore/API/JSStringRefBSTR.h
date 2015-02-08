

#ifndef JSStringRefBSTR_h
#define JSStringRefBSTR_h

#include "JSBase.h"

#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/* COM convenience methods */

JS_EXPORT JSStringRef JSStringCreateWithBSTR(const BSTR string);

JS_EXPORT BSTR JSStringCopyBSTR(const JSStringRef string);
    
#ifdef __cplusplus
}
#endif

#endif /* JSStringRefBSTR_h */
