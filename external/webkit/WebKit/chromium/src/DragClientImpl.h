

#ifndef DragClientImpl_h
#define DragClientImpl_h

#include "DragActions.h"
#include "DragClient.h"

namespace WebCore {
class ClipBoard;
class DragData;
class IntPoint;
class KURL;
}

namespace WebKit {
class WebViewImpl;

class DragClientImpl : public WebCore::DragClient {
public:
    DragClientImpl(WebViewImpl* webView) : m_webView(webView) { }

    virtual void willPerformDragDestinationAction(
        WebCore::DragDestinationAction, WebCore::DragData*);
    virtual void willPerformDragSourceAction(
        WebCore::DragSourceAction, const WebCore::IntPoint&, WebCore::Clipboard*);
    virtual WebCore::DragDestinationAction actionMaskForDrag(WebCore::DragData*);
    virtual WebCore::DragSourceAction dragSourceActionMaskForPoint(
        const WebCore::IntPoint& windowPoint);
    virtual void startDrag(
        WebCore::DragImageRef dragImage,
        const WebCore::IntPoint& dragImageOrigin,
        const WebCore::IntPoint& eventPos,
        WebCore::Clipboard* clipboard,
        WebCore::Frame* frame,
        bool isLinkDrag = false);
    virtual WebCore::DragImageRef createDragImageForLink(
        WebCore::KURL&, const WebCore::String& label, WebCore::Frame*);
    virtual void dragControllerDestroyed();

private:
    WebViewImpl* m_webView;
};

} // namespace WebKit

#endif
