

#import <WebCore/DragClient.h>

@class WebView;

class WebDragClient : public WebCore::DragClient {
public:
    WebDragClient(WebView*);
    virtual void willPerformDragDestinationAction(WebCore::DragDestinationAction, WebCore::DragData*);
    virtual void willPerformDragSourceAction(WebCore::DragSourceAction, const WebCore::IntPoint&, WebCore::Clipboard*);
    virtual WebCore::DragDestinationAction actionMaskForDrag(WebCore::DragData*);
    virtual void dragControllerDestroyed();
    virtual WebCore::DragSourceAction dragSourceActionMaskForPoint(const WebCore::IntPoint& windowPoint);
    virtual void startDrag(WebCore::DragImageRef dragImage, const WebCore::IntPoint& dragPos, const WebCore::IntPoint& eventPos, WebCore::Clipboard*, WebCore::Frame*, bool linkDrag);
    virtual WebCore::DragImageRef createDragImageForLink(WebCore::KURL& url, const WebCore::String& label, WebCore::Frame*);
    virtual void declareAndWriteDragImage(NSPasteboard*, DOMElement*, NSURL*, NSString*, WebCore::Frame*);
private:
    WebView* m_webView;
};
