

#include <WebCore/ContextMenuClient.h>
#include <wtf/Forward.h>

namespace WebCore {
    class String;
}
class WebView;

class WebContextMenuClient : public WebCore::ContextMenuClient {
public:
    WebContextMenuClient(WebView*);

    virtual void contextMenuDestroyed();

    virtual HMENU getCustomMenuFromDefaultItems(WebCore::ContextMenu*);
    virtual void contextMenuItemSelected(WebCore::ContextMenuItem*, const WebCore::ContextMenu*);
    
    virtual void downloadURL(const WebCore::KURL&);
    virtual void searchWithGoogle(const WebCore::Frame*);
    virtual void lookUpInDictionary(WebCore::Frame*);
    virtual void speak(const WebCore::String&);
    virtual void stopSpeaking();
    virtual bool isSpeaking();

private:
    WebView* m_webView;
};
