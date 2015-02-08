

#ifndef ContextMenuClientImpl_h
#define ContextMenuClientImpl_h

#include "ContextMenuClient.h"

namespace WebKit {

class WebViewImpl;
struct WebContextMenuData;

class ContextMenuClientImpl : public  WebCore::ContextMenuClient {
public:
    ContextMenuClientImpl(WebViewImpl* webView) : m_webView(webView) {}
    virtual ~ContextMenuClientImpl() {}
    virtual void copyImageToClipboard(const WebCore::HitTestResult&) {}
    virtual void contextMenuDestroyed() {}
    virtual void contextMenuItemSelected(WebCore::ContextMenuItem*, const WebCore::ContextMenu*) {}
    virtual void downloadURL(const WebCore::KURL&) {}
    virtual WebCore::PlatformMenuDescription getCustomMenuFromDefaultItems(WebCore::ContextMenu*);
    virtual bool isSpeaking() { return false; }
    virtual void lookUpInDictionary(WebCore::Frame*) {}
    virtual void searchWithGoogle(const WebCore::Frame*) {}
    virtual bool shouldIncludeInspectElementItem() { return false; }
    virtual void speak(const WebCore::String&) {}
    virtual void stopSpeaking() {}
private:
    void populateCustomMenuItems(WebCore::ContextMenu*, WebContextMenuData*);
    WebViewImpl* m_webView;
};

} // namespace WebKit

#endif // ContextMenuClientImpl_h
