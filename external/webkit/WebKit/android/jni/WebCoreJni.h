

#ifndef ANDROID_WEBKIT_WEBCOREJNI_H
#define ANDROID_WEBKIT_WEBCOREJNI_H

#include "PlatformString.h"
#include <jni.h>

namespace android {

// A helper class that automatically deletes the local reference to the jobject
// returned from getRealObject.
class AutoJObject {
public:
    ~AutoJObject() {
        if (m_obj)
            m_env->DeleteLocalRef(m_obj);
    }
    jobject get() const {
        return m_obj;
    }
    // Releases the local reference to the caller. The caller *must* delete the
    // local reference when it is done with it.
    jobject release() {
        jobject obj = m_obj;
        m_obj = 0;
        return obj;
    }
    JNIEnv* env() const {
        return m_env;
    }
private:
    AutoJObject(JNIEnv* env, jobject obj)
        : m_env(env)
        , m_obj(obj) {}
    JNIEnv* m_env;
    jobject m_obj;
    friend AutoJObject getRealObject(JNIEnv*, jobject);
};

// Get the real object stored in the weak reference returned as an
// AutoJObject.
AutoJObject getRealObject(JNIEnv*, jobject);

// Helper method for check java exceptions. Returns true if an exception
// occurred and logs the exception.
bool checkException(JNIEnv* env);

// Create a WebCore::String object from a jstring object.
WebCore::String to_string(JNIEnv* env, jstring str);

}

#endif
