

#ifndef JSContextRef_h
#define JSContextRef_h

#include <JavaScriptCore/JSObjectRef.h>
#include <JavaScriptCore/JSValueRef.h>
#include <JavaScriptCore/WebKitAvailability.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

JS_EXPORT JSContextGroupRef JSContextGroupCreate() AVAILABLE_IN_WEBKIT_VERSION_4_0;

JS_EXPORT JSContextGroupRef JSContextGroupRetain(JSContextGroupRef group) AVAILABLE_IN_WEBKIT_VERSION_4_0;

JS_EXPORT void JSContextGroupRelease(JSContextGroupRef group) AVAILABLE_IN_WEBKIT_VERSION_4_0;

JS_EXPORT JSGlobalContextRef JSGlobalContextCreate(JSClassRef globalObjectClass) AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER;

JS_EXPORT JSGlobalContextRef JSGlobalContextCreateInGroup(JSContextGroupRef group, JSClassRef globalObjectClass) AVAILABLE_IN_WEBKIT_VERSION_4_0;

JS_EXPORT JSGlobalContextRef JSGlobalContextRetain(JSGlobalContextRef ctx);

JS_EXPORT void JSGlobalContextRelease(JSGlobalContextRef ctx);

JS_EXPORT JSObjectRef JSContextGetGlobalObject(JSContextRef ctx);

JS_EXPORT JSContextGroupRef JSContextGetGroup(JSContextRef ctx) AVAILABLE_IN_WEBKIT_VERSION_4_0;

#ifdef __cplusplus
}
#endif

#endif /* JSContextRef_h */
