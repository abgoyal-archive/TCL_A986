

#import <WebCore/ContextMenuClient.h>

@class WebView;

class WebContextMenuClient : public WebCore::ContextMenuClient {
public:
    WebContextMenuClient(WebView *webView);
    
    virtual void contextMenuDestroyed();
    
    virtual NSMutableArray* getCustomMenuFromDefaultItems(WebCore::ContextMenu*);
    virtual void contextMenuItemSelected(WebCore::ContextMenuItem*, const WebCore::ContextMenu*);
    
    virtual void downloadURL(const WebCore::KURL&);
    virtual void searchWithGoogle(const WebCore::Frame*);
    virtual void lookUpInDictionary(WebCore::Frame*);
    virtual bool isSpeaking();
    virtual void speak(const WebCore::String&);
    virtual void stopSpeaking();
    virtual void searchWithSpotlight();

    WebView *webView() { return m_webView; }
        
private:
    WebView *m_webView;
};
