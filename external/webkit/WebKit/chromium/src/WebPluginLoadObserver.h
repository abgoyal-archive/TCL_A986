

#ifndef WebPluginLoadObserver_h
#define WebPluginLoadObserver_h

#include "../public/WebURL.h"

namespace WebKit {

class WebPluginContainerImpl;
struct WebURLError;

class WebPluginLoadObserver {
public:
    WebPluginLoadObserver(WebPluginContainerImpl* pluginContainer,
                          const WebURL& notifyURL, void* notifyData)
        : m_pluginContainer(pluginContainer)
        , m_notifyURL(notifyURL)
        , m_notifyData(notifyData)
    {
    }

    ~WebPluginLoadObserver();

    const WebURL& url() const { return m_notifyURL; }

    void clearPluginContainer() { m_pluginContainer = 0; }
    void didFinishLoading();
    void didFailLoading(const WebURLError&);

private:
    WebPluginContainerImpl* m_pluginContainer;
    WebURL m_notifyURL;
    void* m_notifyData;
};

} // namespace WebKit

#endif
