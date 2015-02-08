

#ifndef WebPluginHalterClient_h
#define WebPluginHalterClient_h

#include <WebCore/PluginHalterClient.h>

namespace WebCore {
    class Node;
    class String;
}

class WebView;

class WebPluginHalterClient : public WebCore::PluginHalterClient {
public:
    WebPluginHalterClient(WebView* webView);

    virtual bool shouldHaltPlugin(WebCore::Node* n, bool isWindowed, const WebCore::String& pluginName) const;
    virtual bool enabled() const;

private:
    WebView* m_webView;
};

#endif // WebPluginHalterClient_h
