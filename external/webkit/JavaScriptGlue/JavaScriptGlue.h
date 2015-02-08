

#ifndef JAVASCRIPTGLUE_H
#define JAVASCRIPTGLUE_H

#ifndef __CORESERVICES__
#include <CoreServices/CoreServices.h>
#endif

#ifndef DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER
#define DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* typedefs/structs */
typedef enum {
    kJSFlagNone = 0,
    kJSFlagDebug = 1 << 0,
    kJSFlagConvertAssociativeArray = 1 << 1 /* associative arrays will be converted to dictionaries */
} JSFlags;

typedef struct OpaqueJSTypeRef *JSTypeRef DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef JSTypeRef JSObjectRef DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef JSTypeRef JSRunRef DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef CFTypeID JSTypeID DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

typedef void (*JSObjectDisposeProcPtr)(void *data) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef CFArrayRef (*JSObjectCopyPropertyNamesProcPtr)(void *data) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef JSObjectRef (*JSObjectCopyPropertyProcPtr)(void *data, CFStringRef propertyName) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef void (*JSObjectSetPropertyProcPtr)(void *data, CFStringRef propertyName, JSObjectRef jsValue) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef JSObjectRef (*JSObjectCallFunctionProcPtr)(void *data, JSObjectRef thisObj, CFArrayRef args) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef CFTypeRef (*JSObjectCopyCFValueProcPtr)(void *data) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
typedef UInt8 (*JSObjectEqualProcPtr)(void *data1, void *data2) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

struct JSObjectCallBacks {
    JSObjectDisposeProcPtr dispose;
    JSObjectEqualProcPtr equal;
    JSObjectCopyCFValueProcPtr copyCFValue;
    JSObjectCopyPropertyProcPtr copyProperty;
    JSObjectSetPropertyProcPtr setProperty;
    JSObjectCallFunctionProcPtr callFunction;
    JSObjectCopyPropertyNamesProcPtr copyPropertyNames;
} DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

typedef struct JSObjectCallBacks JSObjectCallBacks, *JSObjectCallBacksPtr DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

void JSSetCFNull(CFTypeRef nullRef) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFTypeRef JSGetCFNull(void) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

JSTypeRef JSRetain(JSTypeRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
void JSRelease(JSTypeRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSTypeID JSGetTypeID(JSTypeRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFIndex JSGetRetainCount(JSTypeRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFStringRef JSCopyDescription(JSTypeRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
UInt8 JSEqual(JSTypeRef ref1, JSTypeRef ref2) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

JSObjectRef JSObjectCreate(void *data, JSObjectCallBacksPtr callBacks) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSObjectRef JSObjectCreateWithCFType(CFTypeRef inRef) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFTypeRef JSObjectCopyCFValue(JSObjectRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
void *JSObjectGetData(JSObjectRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

CFArrayRef JSObjectCopyPropertyNames(JSObjectRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSObjectRef JSObjectCopyProperty(JSObjectRef ref, CFStringRef propertyName) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
void JSObjectSetProperty(JSObjectRef ref, CFStringRef propertyName, JSObjectRef value) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSObjectRef JSObjectCallFunction(JSObjectRef ref, JSObjectRef thisObj, CFArrayRef args) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

JSRunRef JSRunCreate(CFStringRef jsSource, JSFlags inFlags) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFStringRef JSRunCopySource(JSRunRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSObjectRef JSRunCopyGlobalObject(JSRunRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
JSObjectRef JSRunEvaluate(JSRunRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
bool JSRunCheckSyntax(JSRunRef ref) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

void JSCollect(void) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

void JSTypeGetCFArrayCallBacks(CFArrayCallBacks *outCallBacks) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

CFMutableArrayRef JSCreateCFArrayFromJSArray(CFArrayRef array) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
CFMutableArrayRef JSCreateJSArrayFromCFArray(CFArrayRef array) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

void JSLockInterpreter(void) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;
void JSUnlockInterpreter(void) DEPRECATED_IN_MAC_OS_X_VERSION_10_5_AND_LATER;

#ifdef __cplusplus
}
#endif

#endif
