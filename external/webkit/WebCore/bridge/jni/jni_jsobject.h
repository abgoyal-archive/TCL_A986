

#ifndef JAVASCRIPTCORE_BINDINGS_JNI_JSOBJECT_H
#define JAVASCRIPTCORE_BINDINGS_JNI_JSOBJECT_H

#if ENABLE(MAC_JAVA_BRIDGE)

#include <CoreFoundation/CoreFoundation.h>
#include <JavaVM/jni.h>
#include <runtime/JSValue.h>
#include <wtf/RefPtr.h>

#define jlong_to_ptr(a) ((void*)(uintptr_t)(a))
#define jlong_to_impptr(a) (static_cast<JSC::JSObject*>(((void*)(uintptr_t)(a))))
#define ptr_to_jlong(a) ((jlong)(uintptr_t)(a))

namespace JSC {

class ArgList;
class ExecState;
class JSObject;
class MarkedArgumentBuffer;

namespace Bindings {

class RootObject;

enum JSObjectCallType {
    CreateNative,
    Call,
    Eval,
    GetMember,
    SetMember,
    RemoveMember,
    GetSlot,
    SetSlot,
    ToString,
    Finalize
};

struct JSObjectCallContext
{
    JSObjectCallType type;
    jlong nativeHandle;
    jstring string;
    jobjectArray args;
    jint index;
    jobject value;
    CFRunLoopRef originatingLoop;
    jvalue result;
};

class JavaJSObject
{
public:
    JavaJSObject(jlong nativeHandle);
    
    static jlong createNative(jlong nativeHandle);
    jobject call(jstring methodName, jobjectArray args) const;
    jobject eval(jstring script) const;
    jobject getMember(jstring memberName) const;
    void setMember(jstring memberName, jobject value) const;
    void removeMember(jstring memberName) const;
    jobject getSlot(jint index) const;
    void setSlot(jint index, jobject value) const;
    jstring toString() const;
    void finalize() const;
    
    static jvalue invoke(JSObjectCallContext*);

    jobject convertValueToJObject(JSValue) const;
    JSValue convertJObjectToValue(ExecState*, jobject) const;
    void getListFromJArray(ExecState*, jobjectArray, MarkedArgumentBuffer&) const;
    
    RootObject* rootObject() const;
    
    // Must be called from the thread that will be used to access JavaScript.
    static void initializeJNIThreading();
private:
    RefPtr<RootObject> _rootObject;
    JSObject* _imp;
};


} // namespace Bindings

} // namespace JSC

extern "C" {

// The Java VM calls these functions to handle calls to methods in Java's JSObject class.
jlong KJS_JSCreateNativeJSObject(JNIEnv*, jclass, jstring jurl, jlong nativeHandle, jboolean ctx);
void KJS_JSObject_JSFinalize(JNIEnv*, jclass, jlong nativeJSObject);
jobject KJS_JSObject_JSObjectCall(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jstring methodName, jobjectArray args, jboolean ctx);
jobject KJS_JSObject_JSObjectEval(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jstring jscript, jboolean ctx);
jobject KJS_JSObject_JSObjectGetMember(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jstring jname, jboolean ctx);
void KJS_JSObject_JSObjectSetMember(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jstring jname, jobject value, jboolean ctx);
void KJS_JSObject_JSObjectRemoveMember(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jstring jname, jboolean ctx);
jobject KJS_JSObject_JSObjectGetSlot(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jint jindex, jboolean ctx);
void KJS_JSObject_JSObjectSetSlot(JNIEnv*, jclass, jlong nativeJSObject, jstring jurl, jint jindex, jobject value, jboolean ctx);
jstring KJS_JSObject_JSObjectToString(JNIEnv*, jclass, jlong nativeJSObject);

}

#endif // ENABLE(MAC_JAVA_BRIDGE)

#endif // JAVASCRIPTCORE_BINDINGS_JNI_JSOBJECT_H
