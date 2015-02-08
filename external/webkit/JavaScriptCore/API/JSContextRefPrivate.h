

#ifndef JSContextRefPrivate_h
#define JSContextRefPrivate_h

#include <JavaScriptCore/JSObjectRef.h>
#include <JavaScriptCore/JSValueRef.h>
#include <JavaScriptCore/WebKitAvailability.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

JS_EXPORT JSGlobalContextRef JSContextGetGlobalContext(JSContextRef ctx);

#ifdef __cplusplus
}
#endif

#endif /* JSContextRefPrivate_h */
