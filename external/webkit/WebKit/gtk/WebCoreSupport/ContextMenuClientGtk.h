

#ifndef ContextMenuClientGtk_h
#define ContextMenuClientGtk_h

#include "ContextMenuClient.h"

typedef struct _WebKitWebView WebKitWebView;

namespace WebCore {
    class ContextMenu;
}

namespace WebKit {

    class ContextMenuClient : public WebCore::ContextMenuClient
    {
    public:
        ContextMenuClient(WebKitWebView*);

        virtual void contextMenuDestroyed();

        virtual WebCore::PlatformMenuDescription getCustomMenuFromDefaultItems(WebCore::ContextMenu*);
        virtual void contextMenuItemSelected(WebCore::ContextMenuItem*, const WebCore::ContextMenu*);

        virtual void downloadURL(const WebCore::KURL& url);
        virtual void copyImageToClipboard(const WebCore::HitTestResult&);
        virtual void searchWithGoogle(const WebCore::Frame*);
        virtual void lookUpInDictionary(WebCore::Frame*);
        virtual void speak(const WebCore::String&);
        virtual void stopSpeaking();
        virtual bool isSpeaking();

    private:
        WebKitWebView* m_webView;
    };
}

#endif
