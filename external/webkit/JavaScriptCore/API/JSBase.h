

#ifndef JSBase_h
#define JSBase_h

#ifndef __cplusplus
#include <stdbool.h>
#endif

/* JavaScript engine interface */

/*! @typedef JSContextGroupRef A group that associates JavaScript contexts with one another. Contexts in the same group may share and exchange JavaScript objects. */
typedef const struct OpaqueJSContextGroup* JSContextGroupRef;

/*! @typedef JSContextRef A JavaScript execution context. Holds the global object and other execution state. */
typedef const struct OpaqueJSContext* JSContextRef;

/*! @typedef JSGlobalContextRef A global JavaScript execution context. A JSGlobalContext is a JSContext. */
typedef struct OpaqueJSContext* JSGlobalContextRef;

/*! @typedef JSStringRef A UTF16 character buffer. The fundamental string representation in JavaScript. */
typedef struct OpaqueJSString* JSStringRef;

/*! @typedef JSClassRef A JavaScript class. Used with JSObjectMake to construct objects with custom behavior. */
typedef struct OpaqueJSClass* JSClassRef;

/*! @typedef JSPropertyNameArrayRef An array of JavaScript property names. */
typedef struct OpaqueJSPropertyNameArray* JSPropertyNameArrayRef;

/*! @typedef JSPropertyNameAccumulatorRef An ordered set used to collect the names of a JavaScript object's properties. */
typedef struct OpaqueJSPropertyNameAccumulator* JSPropertyNameAccumulatorRef;


/* JavaScript data types */

/*! @typedef JSValueRef A JavaScript value. The base type for all JavaScript values, and polymorphic functions on them. */
typedef const struct OpaqueJSValue* JSValueRef;

/*! @typedef JSObjectRef A JavaScript object. A JSObject is a JSValue. */
typedef struct OpaqueJSValue* JSObjectRef;

/* JavaScript symbol exports */

#undef JS_EXPORT
#if defined(JS_NO_EXPORT)
    #define JS_EXPORT
#elif defined(__GNUC__) && !defined(__CC_ARM) && !defined(__ARMCC__)
    #define JS_EXPORT __attribute__((visibility("default")))
#elif defined(WIN32) || defined(_WIN32) || defined(_WIN32_WCE)
    #if defined(BUILDING_JavaScriptCore) || defined(BUILDING_WTF)
        #define JS_EXPORT __declspec(dllexport)
    #else
        #define JS_EXPORT __declspec(dllimport)
    #endif
#else
    #define JS_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Script Evaluation */

JS_EXPORT JSValueRef JSEvaluateScript(JSContextRef ctx, JSStringRef script, JSObjectRef thisObject, JSStringRef sourceURL, int startingLineNumber, JSValueRef* exception);

JS_EXPORT bool JSCheckScriptSyntax(JSContextRef ctx, JSStringRef script, JSStringRef sourceURL, int startingLineNumber, JSValueRef* exception);

JS_EXPORT void JSGarbageCollect(JSContextRef ctx);

#ifdef __cplusplus
}
#endif

#endif /* JSBase_h */
