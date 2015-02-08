

#ifndef JSProfiler_h
#define JSProfiler_h

#include <JavaScriptCore/JSBase.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

JS_EXPORT void JSStartProfiling(JSContextRef ctx, JSStringRef title);

JS_EXPORT void JSEndProfiling(JSContextRef ctx, JSStringRef title);

#ifdef __cplusplus
}
#endif

#endif /* JSProfiler_h */
