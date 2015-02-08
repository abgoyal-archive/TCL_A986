

#include <WebCore/DragClient.h>

class WebView;

class WebDragClient : public WebCore::DragClient {
public:
    WebDragClient(WebView*);
    virtual WebCore::DragDestinationAction actionMaskForDrag(WebCore::DragData*);
    virtual void willPerformDragDestinationAction(WebCore::DragDestinationAction, WebCore::DragData*);
    virtual void dragControllerDestroyed();
    virtual WebCore::DragSourceAction dragSourceActionMaskForPoint(const WebCore::IntPoint&);
    virtual void willPerformDragSourceAction(WebCore::DragSourceAction, const WebCore::IntPoint&, WebCore::Clipboard*);
    virtual void startDrag(WebCore::DragImageRef, const WebCore::IntPoint&, const WebCore::IntPoint&, WebCore::Clipboard*, WebCore::Frame*, bool);
    virtual WebCore::DragImageRef createDragImageForLink(WebCore::KURL&, const WebCore::String&, WebCore::Frame*);
private:
    WebView* m_webView;
};

