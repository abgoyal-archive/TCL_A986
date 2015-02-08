

#include "config.h"
#include "MyJavaVM.h"

#include <JNIUtility.h>
#include <jni.h>

static JNIEnv* s_env;
static JavaVM* s_jvm;

// JavaVM functions
jint vm_attachCurrentThread(JavaVM*, JNIEnv** env, void*) {
    *env = s_env;
    return JNI_OK;
}

// JNIEnv functions
jobject env_callObjectMethodV(JNIEnv*, jobject, jmethodID, va_list) {
    return MY_JOBJECT;
}
void env_callVoidMethodV(JNIEnv*, jobject, jmethodID, va_list) {}
void env_deleteRef(JNIEnv*, jobject) {}
jboolean env_exceptionCheck(JNIEnv*) {
    return false;
}
jclass env_findClass(JNIEnv*, const char*) {
    return (jclass) 1;
}
jbyte* env_getByteArrayElements(JNIEnv*, jbyteArray, jboolean*) {
    return NULL;
}
jmethodID env_getMethodID(JNIEnv*, jclass, const char*, const char*) {
    return (jmethodID) 1;
}
jclass env_getObjectClass(JNIEnv*, jobject) {
    return (jclass) 1;
}
static const char* s_fakeString = "Fake Java String";
const jchar* env_getStringChars(JNIEnv*, jstring, jboolean* isCopy) {
    if (isCopy)
        *isCopy = false;
    return (const jchar*)s_fakeString;
}
jsize env_getStringLength(JNIEnv*, jstring) {
    return sizeof(s_fakeString) - 1;
}
jbyteArray env_newByteArray(JNIEnv*, jsize) {
    return (jbyteArray) 1;
}
jobject env_newRef(JNIEnv*, jobject obj) {
    return obj;
}
jobject env_newObjectV(JNIEnv*, jclass, jmethodID, va_list) {
    return MY_JOBJECT;
}
jstring env_newString(JNIEnv*, const jchar*, jsize) {
    return (jstring) 1;
}
void env_releaseByteArrayElements(JNIEnv*, jbyteArray, jbyte*, jint) {}
void env_releaseStringChars(JNIEnv*, jstring, const jchar*) {}
void env_setByteArrayRegion(JNIEnv*, jbyteArray, jsize, jsize, const jbyte*) {}
void env_setIntField(JNIEnv*, jobject, jfieldID, jint) {}

void InitializeJavaVM() {
    // First, create the fake vm
    s_jvm = new JavaVM;
    JNIInvokeInterface* i = new JNIInvokeInterface;
    memset(i, 0, sizeof(JNIInvokeInterface));
    s_jvm->functions = i;

    // Now, assign the functions of the vm to our fake ones.
    i->AttachCurrentThread      = vm_attachCurrentThread;

    // Create the fake env next
    s_env = new JNIEnv;
    JNINativeInterface* n = new JNINativeInterface;
    memset(n, 0, sizeof(JNINativeInterface));
    s_env->functions = n;

    // Point the functions we care about to out fake ones.
    n->CallObjectMethodV        = env_callObjectMethodV;
    n->CallVoidMethodV          = env_callVoidMethodV;
    n->DeleteLocalRef           = env_deleteRef;
    n->DeleteGlobalRef          = env_deleteRef;
    n->DeleteWeakGlobalRef      = env_deleteRef;
    n->ExceptionCheck           = env_exceptionCheck;
    n->FindClass                = env_findClass;
    n->GetByteArrayElements     = env_getByteArrayElements;
    n->GetMethodID              = env_getMethodID;
    n->GetObjectClass           = env_getObjectClass;
    n->GetStringChars           = env_getStringChars;
    n->GetStringLength          = env_getStringLength;
    n->NewByteArray             = env_newByteArray;
    n->NewLocalRef              = env_newRef;
    n->NewGlobalRef             = env_newRef;
    n->NewWeakGlobalRef         = env_newRef;
    n->NewObjectV               = env_newObjectV;
    n->NewString                = env_newString;
    n->ReleaseByteArrayElements = env_releaseByteArrayElements;
    n->ReleaseStringChars       = env_releaseStringChars;
    n->SetByteArrayRegion       = env_setByteArrayRegion;
    n->SetIntField              = env_setIntField;

    // Tell WebCore about the vm
    JSC::Bindings::setJavaVM(s_jvm);
}
