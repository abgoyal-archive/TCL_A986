

#define LOG_TAG "webcoreglue"

#include "config.h"
#include "WebCoreJni.h"

#include "NotImplemented.h"
#include <JNIUtility.h>
#include <jni.h>
#include <utils/Log.h>

namespace android {

AutoJObject getRealObject(JNIEnv* env, jobject obj)
{
    jobject real = env->NewLocalRef(obj);
    LOG_ASSERT(real, "The real object has been deleted!");
    return AutoJObject(env, real);
}

bool checkException(JNIEnv* env)
{
    if (env->ExceptionCheck() != 0)
    {
        LOGE("*** Uncaught exception returned from Java call!\n");
        env->ExceptionDescribe();
        return true;
    }
    return false;
}

// This method is safe to call from the ui thread and the WebCore thread.
WebCore::String to_string(JNIEnv* env, jstring str)
{
    if (!str || !env)
        return WebCore::String();
    const jchar* s = env->GetStringChars(str, NULL);
    if (!s)
        return WebCore::String();
    WebCore::String ret(s, env->GetStringLength(str));
    env->ReleaseStringChars(str, s);
    checkException(env);
    return ret;
}

}
