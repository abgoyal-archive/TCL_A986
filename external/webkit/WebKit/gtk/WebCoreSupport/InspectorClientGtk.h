

#ifndef InspectorClientGtk_h
#define InspectorClientGtk_h

#include "InspectorClient.h"
#include "webkitwebview.h"
#include "webkitwebinspector.h"

namespace WebCore {
    class Node;
    class Page;
    class String;
}

namespace WebKit {

    class InspectorClient : public WebCore::InspectorClient {
    public:
        InspectorClient(WebKitWebView* webView);

        virtual void inspectorDestroyed();
        void webViewDestroyed();

        virtual WebCore::Page* createPage();

        virtual WebCore::String localizedStringsURL();

        virtual WebCore::String hiddenPanels();

        virtual void showWindow();
        virtual void closeWindow();

        virtual void attachWindow();
        virtual void detachWindow();

        virtual void setAttachedWindowHeight(unsigned height);

        virtual void highlight(WebCore::Node*);
        virtual void hideHighlight();
        virtual void inspectedURLChanged(const WebCore::String& newURL);

        virtual void populateSetting(const WebCore::String& key, WebCore::String* value);
        virtual void storeSetting(const WebCore::String& key, const WebCore::String& value);

        virtual void inspectorWindowObjectCleared();

    private:
        WebKitWebView* m_webView;
        WebKitWebView* m_inspectedWebView;
        WebKitWebInspector* m_webInspector;
    };
}

#endif
