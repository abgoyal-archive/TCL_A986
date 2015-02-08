

#ifndef JNIUtilityPrivate_h
#define JNIUtilityPrivate_h

#if ENABLE(MAC_JAVA_BRIDGE)

#include "JNIUtility.h"
#include <runtime/JSValue.h>

namespace JSC {

class ExecState;
class JSObject;

namespace Bindings {

jvalue convertValueToJValue(ExecState*, JSValue, JNIType, const char* javaClassName);
bool dispatchJNICall(ExecState*, const void* targetAppletView, jobject obj, bool isStatic, JNIType returnType, jmethodID methodID, jvalue* args, jvalue& result, const char* callingURL, JSValue& exceptionDescription);

} // namespace Bindings

} // namespace JSC

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JNIUtilityPrivate_h
