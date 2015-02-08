

#include "config.h"
#include "PluginData.h"

namespace WebCore {

PluginData::PluginData(const Page* page)
    : m_page(page)
{
    initPlugins();

    for (unsigned i = 0; i < m_plugins.size(); ++i) {
        const PluginInfo* plugin = m_plugins[i];
        for (unsigned j = 0; j < plugin->mimes.size(); ++j)
            m_mimes.append(plugin->mimes[j]);
    }
}

PluginData::~PluginData()
{
    deleteAllValues(m_plugins);
    deleteAllValues(m_mimes);
}

bool PluginData::supportsMimeType(const String& mimeType) const
{
    for (unsigned i = 0; i < m_mimes.size(); ++i)
        if (m_mimes[i]->type == mimeType)
            return true;
    return false;
}

String PluginData::pluginNameForMimeType(const String& mimeType) const
{
    for (unsigned i = 0; i < m_mimes.size(); ++i)
        if (m_mimes[i]->type == mimeType)
            return m_mimes[i]->plugin->name;
    return String();
}

}
