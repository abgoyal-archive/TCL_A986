

#import <WebCore/PluginHalterClient.h>

namespace WebCore {
    class Node;
    class String;
}

@class WebView;

class WebPluginHalterClient : public WebCore::PluginHalterClient {
public:
    WebPluginHalterClient(WebView *);
    
    virtual bool shouldHaltPlugin(WebCore::Node*, bool, const WebCore::String&) const;
    virtual bool enabled() const;
    
private:
    WebView *m_webView;
};
