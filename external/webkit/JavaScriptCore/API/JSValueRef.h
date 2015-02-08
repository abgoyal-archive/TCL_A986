

#ifndef JSValueRef_h
#define JSValueRef_h

#include <JavaScriptCore/JSBase.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef enum {
    kJSTypeUndefined,
    kJSTypeNull,
    kJSTypeBoolean,
    kJSTypeNumber,
    kJSTypeString,
    kJSTypeObject
} JSType;

#ifdef __cplusplus
extern "C" {
#endif

JS_EXPORT JSType JSValueGetType(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsUndefined(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsNull(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsBoolean(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsNumber(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsString(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsObject(JSContextRef ctx, JSValueRef value);

JS_EXPORT bool JSValueIsObjectOfClass(JSContextRef ctx, JSValueRef value, JSClassRef jsClass);

/* Comparing values */

JS_EXPORT bool JSValueIsEqual(JSContextRef ctx, JSValueRef a, JSValueRef b, JSValueRef* exception);

JS_EXPORT bool JSValueIsStrictEqual(JSContextRef ctx, JSValueRef a, JSValueRef b);

JS_EXPORT bool JSValueIsInstanceOfConstructor(JSContextRef ctx, JSValueRef value, JSObjectRef constructor, JSValueRef* exception);

/* Creating values */

JS_EXPORT JSValueRef JSValueMakeUndefined(JSContextRef ctx);

JS_EXPORT JSValueRef JSValueMakeNull(JSContextRef ctx);

JS_EXPORT JSValueRef JSValueMakeBoolean(JSContextRef ctx, bool boolean);

JS_EXPORT JSValueRef JSValueMakeNumber(JSContextRef ctx, double number);

JS_EXPORT JSValueRef JSValueMakeString(JSContextRef ctx, JSStringRef string);

/* Converting to primitive values */

JS_EXPORT bool JSValueToBoolean(JSContextRef ctx, JSValueRef value);

JS_EXPORT double JSValueToNumber(JSContextRef ctx, JSValueRef value, JSValueRef* exception);

JS_EXPORT JSStringRef JSValueToStringCopy(JSContextRef ctx, JSValueRef value, JSValueRef* exception);

JS_EXPORT JSObjectRef JSValueToObject(JSContextRef ctx, JSValueRef value, JSValueRef* exception);

/* Garbage collection */
JS_EXPORT void JSValueProtect(JSContextRef ctx, JSValueRef value);

JS_EXPORT void JSValueUnprotect(JSContextRef ctx, JSValueRef value);

#ifdef __cplusplus
}
#endif

#endif /* JSValueRef_h */
