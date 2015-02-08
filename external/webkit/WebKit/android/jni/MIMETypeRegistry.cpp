

#define LOG_TAG "WebCore"

#include "config.h"
#include "MIMETypeRegistry.h"

#include "PlatformString.h"
#include "WebCoreJni.h"

#include <JNIUtility.h>
#include <jni.h>
#include <utils/Log.h>

namespace WebCore {

String MIMETypeRegistry::getMIMETypeForExtension(const String& ext)
{
    JNIEnv* env = JSC::Bindings::getJNIEnv();
    jclass mimeClass = env->FindClass("android/webkit/MimeTypeMap");
    LOG_ASSERT(mimeClass, "Could not find class MimeTypeMap");
    jmethodID mimeTypeFromExtension = env->GetStaticMethodID(mimeClass,
            "mimeTypeFromExtension",
            "(Ljava/lang/String;)Ljava/lang/String;");
    LOG_ASSERT(mimeTypeFromExtension,
            "Could not find method mimeTypeFromExtension");
    jstring extString =
            env->NewString((const jchar*) ext.characters(), ext.length());
    jobject mimeType = env->CallStaticObjectMethod(mimeClass,
            mimeTypeFromExtension, extString);
    String result = android::to_string(env, (jstring) mimeType);
    env->DeleteLocalRef(mimeClass);
    env->DeleteLocalRef(extString);
    env->DeleteLocalRef(mimeType);
    return result;
}

}
