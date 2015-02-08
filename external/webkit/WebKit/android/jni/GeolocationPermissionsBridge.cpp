

#include "config.h"

#include <JNIHelp.h>  // For jniRegisterNativeMethods
#include "GeolocationPermissions.h"
#include "WebCoreJni.h"  // For to_string



namespace android {

static jobject getOrigins(JNIEnv* env, jobject obj)
{
    GeolocationPermissions::OriginSet origins = GeolocationPermissions::getOrigins();

    jclass setClass = env->FindClass("java/util/HashSet");
    jmethodID constructor = env->GetMethodID(setClass, "<init>", "()V");
    jmethodID addMethod = env->GetMethodID(setClass, "add", "(Ljava/lang/Object;)Z");
    jobject set = env->NewObject(setClass, constructor);

    GeolocationPermissions::OriginSet::const_iterator end = origins.end();
    for (GeolocationPermissions::OriginSet::const_iterator iter = origins.begin(); iter != end; ++iter) {
        jstring originString = env->NewString(iter->characters(), iter->length());
        env->CallBooleanMethod(set, addMethod, originString);
        env->DeleteLocalRef(originString);
    }
    return set;
}

static bool getAllowed(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originString = to_string(env, origin);
    return GeolocationPermissions::getAllowed(originString);
}

static void clear(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originString = to_string(env, origin);
    GeolocationPermissions::clear(originString);
}

static void allow(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originString = to_string(env, origin);
    GeolocationPermissions::allow(originString);
}

static void clearAll(JNIEnv* env, jobject obj)
{
    GeolocationPermissions::clearAll();
}

static JNINativeMethod gGeolocationPermissionsMethods[] = {
    { "nativeGetOrigins", "()Ljava/util/Set;",
        (void*) getOrigins },
    { "nativeGetAllowed", "(Ljava/lang/String;)Z",
        (void*) getAllowed },
    { "nativeClear", "(Ljava/lang/String;)V",
        (void*) clear },
    { "nativeAllow", "(Ljava/lang/String;)V",
        (void*) allow },
    { "nativeClearAll", "()V",
        (void*) clearAll }
};

int register_geolocation_permissions(JNIEnv* env)
{
    const char* kGeolocationPermissionsClass = "android/webkit/GeolocationPermissions";
    jclass geolocationPermissions = env->FindClass(kGeolocationPermissionsClass);
    LOG_ASSERT(geolocationPermissions, "Unable to find class");

    return jniRegisterNativeMethods(env, kGeolocationPermissionsClass,
            gGeolocationPermissionsMethods, NELEM(gGeolocationPermissionsMethods));
}

}
