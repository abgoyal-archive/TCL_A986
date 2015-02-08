

#include "config.h"
#include "MimeType.h"

#include "Frame.h"
#include "Page.h"
#include "Plugin.h"
#include "PluginData.h"
#include "Settings.h"

namespace WebCore {

MimeType::MimeType(PassRefPtr<PluginData> pluginData, unsigned index)
    : m_pluginData(pluginData)
    , m_index(index)
{
}

MimeType::~MimeType()
{
}

const String &MimeType::type() const
{
    return m_pluginData->mimes()[m_index]->type;
}

const String &MimeType::suffixes() const
{
    return m_pluginData->mimes()[m_index]->suffixes;
}

const String &MimeType::description() const
{
    return m_pluginData->mimes()[m_index]->desc;
}

PassRefPtr<Plugin> MimeType::enabledPlugin() const
{
    const Page* p = m_pluginData->page();
    if (!p || !p->settings()->arePluginsEnabled())
        return 0;

    const PluginInfo *info = m_pluginData->mimes()[m_index]->plugin;
    const Vector<PluginInfo*>& plugins = m_pluginData->plugins();
    for (size_t i = 0; i < plugins.size(); ++i) {
        if (plugins[i] == info)
            return Plugin::create(m_pluginData.get(), i);
    }
    return 0;
}

} // namespace WebCore
