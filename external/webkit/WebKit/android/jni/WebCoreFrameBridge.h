

// TODO: change name to WebFrame.h

#ifndef WEBFRAME_H
#define WEBFRAME_H

#include "FrameLoaderClient.h"
#include "PlatformBridge.h"
#include "WebCoreRefObject.h"
#include <jni.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    class HistoryItem;
    class Image;
    class Page;
    class RenderPart;
    class ResourceHandle;
    class ResourceLoaderAndroid;
    class ResourceRequest;
}

namespace android {

class WebViewCore;

// one instance of WebFrame per Page for calling into Java's BrowserFrame
class WebFrame : public WebCoreRefObject {
  public:
    WebFrame(JNIEnv* env, jobject obj, jobject historyList, WebCore::Page* page);
    ~WebFrame();

    // helper function
    static WebFrame* getWebFrame(const WebCore::Frame* frame);

    virtual PassRefPtr<WebCore::ResourceLoaderAndroid> startLoadingResource(WebCore::ResourceHandle*,
            const WebCore::ResourceRequest& request, bool mainResource,
            bool synchronous);

    void reportError(int errorCode, const WebCore::String& description,
            const WebCore::String& failingUrl);

    void loadStarted(WebCore::Frame* frame);

    void transitionToCommitted(WebCore::Frame* frame);

    void didFinishLoad(WebCore::Frame* frame);

    void addHistoryItem(WebCore::HistoryItem* item);

    void removeHistoryItem(int index);

    void updateHistoryIndex(int newIndex);

    void setTitle(const WebCore::String& title);

    void windowObjectCleared(WebCore::Frame* frame);

	void setProgress(float newProgress);

    const WebCore::String userAgentForURL(const WebCore::KURL* url);

    void didReceiveIcon(WebCore::Image* icon);

    void didReceiveTouchIconURL(const WebCore::String& url, bool precomposed);
    
    void updateVisitedHistory(const WebCore::KURL& url, bool reload);

    virtual bool canHandleRequest(const WebCore::ResourceRequest& request);

    WebCore::Frame* createWindow(bool dialog, bool userGesture);

    void requestFocus() const;

    void closeWindow(WebViewCore* webViewCore);

    void decidePolicyForFormResubmission(WebCore::FramePolicyFunction func);

    void setUserAgent(WebCore::String userAgent) { mUserAgent = userAgent; }

    WebCore::String getRawResourceFilename(WebCore::PlatformBridge::rawResId) const;

    float density() const;

    /**
     * When the user initiates a click (via trackball, enter-press, or touch),
     * we set mUserInitiatedClick to true.  If a load happens due to this click,
     * then we ask the application if it wants to override
     * the load. Otherwise, we attempt to load the resource internally.
     * We also check it to determine whether or not to allow webkit to request
     * a scroll.  If it was user initated, the scroll is allowed.
     */
    void setUserInitiatedClick(bool userInitiatedClick) { mUserInitiatedClick = userInitiatedClick; }

    bool userInitiatedClick() { return mUserInitiatedClick; }
    
    WebCore::Page* page() const { return mPage; }

private:
    struct JavaBrowserFrame;
    JavaBrowserFrame* mJavaFrame;
    WebCore::Page* mPage;
    WebCore::String mUserAgent;
    bool mUserInitiatedClick;
};

}   // namespace android

#endif // WEBFRAME_H
