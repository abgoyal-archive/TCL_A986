

#import <WebCore/InspectorClient.h>
#import <WebCore/PlatformString.h>

#import <wtf/RetainPtr.h>

#ifdef __OBJC__
@class WebInspectorWindowController;
@class WebView;
#else
class WebInspectorWindowController;
class WebView;
#endif

class WebInspectorClient : public WebCore::InspectorClient {
public:
    WebInspectorClient(WebView *);

    virtual void inspectorDestroyed();

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
    void updateWindowTitle() const;

    WebView *m_webView;
    RetainPtr<WebInspectorWindowController> m_windowController;
    WebCore::String m_inspectedURL;
};
