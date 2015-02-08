

#ifndef JSBasePrivate_h
#define JSBasePrivate_h

#include <JavaScriptCore/JSBase.h>
#include <JavaScriptCore/WebKitAvailability.h>

#ifdef __cplusplus
extern "C" {
#endif

JS_EXPORT void JSReportExtraMemoryCost(JSContextRef ctx, size_t size) AVAILABLE_IN_WEBKIT_VERSION_4_0;

#ifdef __cplusplus
}
#endif

#endif /* JSBasePrivate_h */
