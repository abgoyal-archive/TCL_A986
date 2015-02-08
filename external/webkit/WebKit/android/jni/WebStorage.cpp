

#include "config.h"

#if ENABLE(DATABASE)

#include "JavaSharedClient.h"
#include "KURL.h"
#include "PageGroup.h"
#include "WebCoreJni.h"

#include <JNIHelp.h>
#include <JNIUtility.h>
#include <WebCore/loader/appcache/ApplicationCacheStorage.h>
#include <WebCore/page/SecurityOrigin.h>
#include <WebCore/storage/DatabaseTracker.h>

namespace android {

static jobject GetOrigins(JNIEnv* env, jobject obj)
{
    Vector<RefPtr<WebCore::SecurityOrigin> > coreOrigins;
    WebCore::DatabaseTracker::tracker().origins(coreOrigins);
    Vector<WebCore::KURL> manifestUrls;
    if (WebCore::cacheStorage().manifestURLs(&manifestUrls)) {
        int size = manifestUrls.size();
        for (int i = 0; i < size; ++i) {
            RefPtr<WebCore::SecurityOrigin> manifestOrigin = WebCore::SecurityOrigin::create(manifestUrls[i]);
            if (manifestOrigin.get() == 0)
                continue;
            coreOrigins.append(manifestOrigin);
        }
    }

    jclass setClass = env->FindClass("java/util/HashSet");
    jmethodID cid = env->GetMethodID(setClass, "<init>", "()V");
    jmethodID mid = env->GetMethodID(setClass, "add", "(Ljava/lang/Object;)Z");
    jobject set = env->NewObject(setClass, cid);

    for (unsigned i = 0; i < coreOrigins.size(); ++i) {
        WebCore::SecurityOrigin* origin = coreOrigins[i].get();
        WebCore::String url = origin->toString();
        jstring jUrl = env->NewString(url.characters(), url.length());
        env->CallBooleanMethod(set, mid, jUrl);
        env->DeleteLocalRef(jUrl);
    }

    return set;
}

static unsigned long long GetQuotaForOrigin(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originStr = to_string(env, origin);
    RefPtr<WebCore::SecurityOrigin> securityOrigin = WebCore::SecurityOrigin::createFromString(originStr);
    unsigned long long quota = WebCore::DatabaseTracker::tracker().quotaForOrigin(securityOrigin.get());
    return quota;
}

static unsigned long long GetUsageForOrigin(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originStr = to_string(env, origin);
    RefPtr<WebCore::SecurityOrigin> securityOrigin = WebCore::SecurityOrigin::createFromString(originStr);
    unsigned long long usage = WebCore::DatabaseTracker::tracker().usageForOrigin(securityOrigin.get());
    Vector<WebCore::KURL> manifestUrls;
    if (!WebCore::cacheStorage().manifestURLs(&manifestUrls))
        return usage;
    int size = manifestUrls.size();
    for (int i = 0; i < size; ++i) {
        RefPtr<WebCore::SecurityOrigin> manifestOrigin = WebCore::SecurityOrigin::create(manifestUrls[i]);
        if (manifestOrigin.get() == 0)
            continue;
        if (manifestOrigin->isSameSchemeHostPort(securityOrigin.get())) {
            int64_t cacheSize = 0;
            WebCore::cacheStorage().cacheGroupSize(manifestUrls[i].string(), &cacheSize);
            usage += cacheSize;
        }
    }
    return usage;
}

static void SetQuotaForOrigin(JNIEnv* env, jobject obj, jstring origin, unsigned long long quota)
{
    WebCore::String originStr = to_string(env, origin);
    RefPtr<WebCore::SecurityOrigin> securityOrigin = WebCore::SecurityOrigin::createFromString(originStr);
    WebCore::DatabaseTracker::tracker().setQuota(securityOrigin.get(), quota);
}

static void DeleteOrigin(JNIEnv* env, jobject obj, jstring origin)
{
    WebCore::String originStr = to_string(env, origin);
    RefPtr<WebCore::SecurityOrigin> securityOrigin = WebCore::SecurityOrigin::createFromString(originStr);
    WebCore::DatabaseTracker::tracker().deleteOrigin(securityOrigin.get());

    Vector<WebCore::KURL> manifestUrls;
    if (!WebCore::cacheStorage().manifestURLs(&manifestUrls))
        return;
    int size = manifestUrls.size();
    for (int i = 0; i < size; ++i) {
        RefPtr<WebCore::SecurityOrigin> manifestOrigin = WebCore::SecurityOrigin::create(manifestUrls[i]);
        if (manifestOrigin.get() == 0)
            continue;
        if (manifestOrigin->isSameSchemeHostPort(securityOrigin.get()))
            WebCore::cacheStorage().deleteCacheGroup(manifestUrls[i]);
    }
}

static void DeleteAllData(JNIEnv* env, jobject obj)
{
    WebCore::DatabaseTracker::tracker().deleteAllDatabases();
    WebCore::cacheStorage().empty();
    // FIXME: this is a workaround for eliminating any DOM Storage data (both
    // session and local storage) as there is no functionality inside WebKit at the
    // moment to do it. That functionality is a WIP in https://bugs.webkit.org/show_bug.cgi?id=51878
    // and when that patch lands and we merge it, we should move towards that approach instead.
    WebCore::PageGroup::clearDomStorage();
}

static void SetAppCacheMaximumSize(JNIEnv* env, jobject obj, unsigned long long size)
{
    WebCore::cacheStorage().setMaximumSize(size);
}

static JNINativeMethod gWebStorageMethods[] = {
    { "nativeGetOrigins", "()Ljava/util/Set;",
        (void*) GetOrigins },
    { "nativeGetUsageForOrigin", "(Ljava/lang/String;)J",
        (void*) GetUsageForOrigin },
    { "nativeGetQuotaForOrigin", "(Ljava/lang/String;)J",
        (void*) GetQuotaForOrigin },
    { "nativeSetQuotaForOrigin", "(Ljava/lang/String;J)V",
        (void*) SetQuotaForOrigin },
    { "nativeDeleteOrigin", "(Ljava/lang/String;)V",
        (void*) DeleteOrigin },
    { "nativeDeleteAllData", "()V",
        (void*) DeleteAllData },
    { "nativeSetAppCacheMaximumSize", "(J)V",
        (void*) SetAppCacheMaximumSize }
};

int register_webstorage(JNIEnv* env)
{
    jclass webStorage = env->FindClass("android/webkit/WebStorage");
    LOG_ASSERT(webStorage, "Unable to find class android.webkit.WebStorage");

    return jniRegisterNativeMethods(env, "android/webkit/WebStorage",
            gWebStorageMethods, NELEM(gWebStorageMethods));
}

}

#endif //ENABLE(DATABASE)
