

#include "config.h"
#include "Page.h"

#include "Frame.h"
#include "FrameLoaderClient.h"
#include "FrameView.h"
#include "FloatRect.h"
#include "PluginView.h"
#include <windows.h>

namespace WebCore {

HINSTANCE Page::s_instanceHandle = 0;

void Page::setCanStartPlugins(bool canStartPlugins)
{
    if (m_canStartPlugins == canStartPlugins)
        return;

    m_canStartPlugins = canStartPlugins;

    if (!m_canStartPlugins || m_unstartedPlugins.isEmpty())
        return;

    Vector<PluginView*> unstartedPlugins;
    copyToVector(m_unstartedPlugins, unstartedPlugins);
    m_unstartedPlugins.clear();

    for (size_t i = 0; i < unstartedPlugins.size(); ++i) {
        if (unstartedPlugins[i]->start())
            continue;
        unstartedPlugins[i]->parentFrame()->loader()->client()->dispatchDidFailToStartPlugin(unstartedPlugins[i]);
    }
}

} // namespace WebCore

