

#ifndef WEB_FRAMEVIEW_H
#define WEB_FRAMEVIEW_H

#include "WebCoreViewBridge.h"

namespace WebCore {
    class FrameView;
}

namespace android {
    class WebViewCore;

    class WebFrameView: public WebCoreViewBridge {
    public:
        WebFrameView(WebCore::FrameView* frameView, WebViewCore* webViewCore);
        virtual ~WebFrameView();

        virtual void draw(WebCore::GraphicsContext* ctx,
                const WebCore::IntRect& rect);

        WebViewCore* webViewCore() const {
            return mWebViewCore;
        }

        void setView(WebCore::FrameView* frameView);

        WebCore::FrameView* view() const {
            return mFrameView;
        }

        virtual bool forFrameView() const { return true; }

    private:
        WebCore::FrameView* mFrameView;
        WebViewCore* mWebViewCore;
    };

} // namespace android

#endif // WEB_FRAMEVIEW_H
