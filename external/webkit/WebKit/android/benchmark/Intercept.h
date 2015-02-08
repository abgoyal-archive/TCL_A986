

#ifndef INTERCEPT_H
#define INTERCEPT_H

#include "MyJavaVM.h"
#include "PlatformString.h"
#include "Timer.h"
#include "WebCoreFrameBridge.h"
#include "WebCoreResourceLoader.h"
#include <JNIUtility.h>
#include <wtf/Vector.h>

namespace WebCore {
    class Page;
    class ResourceHandle;
    class ResourceRequest;
}

using namespace android;
using namespace WebCore;
using namespace WTF;

class MyResourceLoader : public WebCoreResourceLoader {
public:
    static PassRefPtr<WebCore::ResourceLoaderAndroid> create(
            ResourceHandle* handle, String url);
    void handleRequest();

private:
    MyResourceLoader(ResourceHandle* handle, String url)
        : WebCoreResourceLoader(JSC::Bindings::getJNIEnv(), MY_JOBJECT)
        , m_handle(handle)
        , m_url(url) {}

    void loadData(const String&);
    void loadFile(const String&);
    ResourceHandle* m_handle;
    String m_url;
};

class MyWebFrame : public WebFrame {
public:
    MyWebFrame(Page* page)
        : WebFrame(JSC::Bindings::getJNIEnv(), MY_JOBJECT, MY_JOBJECT, page)
        , m_timer(this, &MyWebFrame::timerFired) {}

    virtual PassRefPtr<WebCore::ResourceLoaderAndroid> startLoadingResource(
            ResourceHandle* handle, const ResourceRequest& req, bool, bool);

    virtual bool canHandleRequest(const ResourceRequest&) { return true; }

private:
    void timerFired(Timer<MyWebFrame>*);
    Vector<RefPtr<WebCore::ResourceLoaderAndroid> > m_requests;
    Timer<MyWebFrame> m_timer;
};

#endif
