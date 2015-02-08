

#ifndef ANDROID_WEBKIT_RESOURCELOADLISTENER_H
#define ANDROID_WEBKIT_RESOURCELOADLISTENER_H

#include <KURL.h>
#include <ResourceLoaderAndroid.h>
#include <jni.h>

namespace android {

class WebCoreResourceLoader : public WebCore::ResourceLoaderAndroid
{
public:
    static PassRefPtr<WebCore::ResourceLoaderAndroid> create(JNIEnv *env, jobject jLoadListener);
    virtual ~WebCoreResourceLoader();

    /**
     * Call to java to cancel the current load.
     */
    virtual void cancel();

    /**
    * Call to java to download the current load rather than feed it
    * back to WebCore
    */
    virtual void downloadFile();

    virtual void pauseLoad(bool);

    /**
    * Call to java to find out if this URL is in the cache
    */
    static bool willLoadFromCache(const WebCore::KURL& url, int64_t identifier);

    // Native jni functions
    static void SetResponseHeader(JNIEnv*, jobject, jint, jstring, jstring);
    static jint CreateResponse(JNIEnv*, jobject, jstring, jint, jstring,
            jstring, jlong, jstring);
    static void ReceivedResponse(JNIEnv*, jobject, jint);
    static void AddData(JNIEnv*, jobject, jbyteArray, jint);
    static void Finished(JNIEnv*, jobject);
    static jstring RedirectedToUrl(JNIEnv*, jobject, jstring, jstring, jint);
    static void Error(JNIEnv*, jobject, jint, jstring, jstring);

protected:
    WebCoreResourceLoader(JNIEnv *env, jobject jLoadListener);
private:
    jobject     mJLoader;
    bool        mPausedLoad;
};

} // end namespace android

#endif
