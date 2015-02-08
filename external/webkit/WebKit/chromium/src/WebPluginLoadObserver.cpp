

#include "config.h"
#include "WebPluginLoadObserver.h"

#include "WebPlugin.h"
#include "WebPluginContainerImpl.h"

namespace WebKit {

WebPluginLoadObserver::~WebPluginLoadObserver()
{
    if (m_pluginContainer)
        m_pluginContainer->willDestroyPluginLoadObserver(this);
}

void WebPluginLoadObserver::didFinishLoading()
{
    if (m_pluginContainer)
        m_pluginContainer->plugin()->didFinishLoadingFrameRequest(m_notifyURL, m_notifyData);
}

void WebPluginLoadObserver::didFailLoading(const WebURLError& error)
{
    if (m_pluginContainer)
        m_pluginContainer->plugin()->didFailLoadingFrameRequest(m_notifyURL, m_notifyData, error);
}

} // namespace WebKit
