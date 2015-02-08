

#ifndef KJS_BINDINGS_OBJC_UTILITY_H
#define KJS_BINDINGS_OBJC_UTILITY_H

#include <CoreFoundation/CoreFoundation.h>

#include "objc_header.h"
#include <runtime/Error.h>
#include <runtime/JSObject.h>

#ifdef __OBJC__
@class NSString;
#else
class NSString;
#endif

namespace JSC {
namespace Bindings {

typedef union {
    ObjectStructPtr objectValue;
    bool booleanValue;
    char charValue;
    short shortValue;
    int intValue;
    long longValue;
    long long longLongValue;
    float floatValue;
    double doubleValue;
} ObjcValue;

typedef enum {
    ObjcVoidType,
    ObjcObjectType,
    ObjcCharType,
    ObjcUnsignedCharType,
    ObjcShortType,
    ObjcUnsignedShortType,
    ObjcIntType,
    ObjcUnsignedIntType,
    ObjcLongType,
    ObjcUnsignedLongType,
    ObjcLongLongType,
    ObjcUnsignedLongLongType,
    ObjcFloatType,
    ObjcDoubleType,
    ObjcInvalidType
} ObjcValueType;

class RootObject;

ObjcValue convertValueToObjcValue(ExecState*, JSValue, ObjcValueType);
JSValue convertNSStringToString(ExecState* exec, NSString *nsstring);
JSValue convertObjcValueToValue(ExecState*, void* buffer, ObjcValueType, RootObject*);
ObjcValueType objcValueTypeForType(const char *type);

bool convertJSMethodNameToObjc(const char *JSName, char *buffer, size_t bufferSize);

JSObject *throwError(ExecState *, ErrorType, NSString *message);

} // namespace Bindings
} // namespace JSC

#endif
