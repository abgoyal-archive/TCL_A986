

#include "config.h"

#include "WebPluginHalterClient.h"

#include "DOMCoreClasses.h"
#include "WebView.h"
#include <WebCore/Node.h>
#include <wtf/Assertions.h>

using namespace WebCore;

WebPluginHalterClient::WebPluginHalterClient(WebView* webView)
    : m_webView(webView)
{
    ASSERT_ARG(webView, webView);
}

bool WebPluginHalterClient::shouldHaltPlugin(Node* n, bool isWindowed, const String& pluginName) const
{
    ASSERT_ARG(n, n);

    IWebPluginHalterDelegate* d;
    if (FAILED(m_webView->pluginHalterDelegate(&d)) || !d)
        return false;

    COMPtr<IDOMNode> domNode(AdoptCOM, DOMNode::createInstance(n));

    BOOL shouldHalt;
    if (FAILED(d->shouldHaltPlugin(m_webView, domNode.get(), isWindowed, BString(pluginName), &shouldHalt)))
        return false;

    return shouldHalt;
}

bool WebPluginHalterClient::enabled() const
{
    IWebPluginHalterDelegate* d;
    if (FAILED(m_webView->pluginHalterDelegate(&d)) || !d)
        return false;

    return true;
}
