

#ifndef InspectorClientImpl_h
#define InspectorClientImpl_h

#include "InspectorClient.h"
#include "InspectorController.h"
#include <wtf/OwnPtr.h>

namespace WebKit {
class WebViewImpl;

class InspectorClientImpl : public WebCore::InspectorClient {
public:
    InspectorClientImpl(WebViewImpl*);
    ~InspectorClientImpl();

    // InspectorClient methods:
    virtual void inspectorDestroyed();
    virtual WebCore::Page* createPage();
    virtual WebCore::String localizedStringsURL();
    virtual WebCore::String hiddenPanels();
    virtual void showWindow();
    virtual void closeWindow();
    virtual bool windowVisible();
    virtual void attachWindow();
    virtual void detachWindow();
    virtual void setAttachedWindowHeight(unsigned height);
    virtual void highlight(WebCore::Node*);
    virtual void hideHighlight();
    virtual void inspectedURLChanged(const WebCore::String& newURL);
    virtual void populateSetting(
        const WebCore::String& key,
        WebCore::String* value);
    virtual void storeSetting(
        const WebCore::String& key,
        const WebCore::String& value);
    virtual void inspectorWindowObjectCleared();

private:
    void loadSettings();
    void saveSettings();

    // The WebViewImpl of the page being inspected; gets passed to the constructor
    WebViewImpl* m_inspectedWebView;

    typedef HashMap<WebCore::String, WebCore::String> SettingsMap;
    OwnPtr<SettingsMap> m_settings;
};

} // namespace WebKit

#endif
