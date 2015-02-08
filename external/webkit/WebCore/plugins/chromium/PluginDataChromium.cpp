

#include "config.h"
#include "PluginDataChromium.h"

#include "ChromiumBridge.h"

namespace WebCore {

static PluginInfo* clone(const PluginInfo* info)
{
    PluginInfo* result = new PluginInfo();
    result->name = info->name;
    result->desc = info->desc;
    result->file = info->file;
    for (size_t i = 0; i < info->mimes.size(); ++i) {
        MimeClassInfo* mime = new MimeClassInfo();
        mime->type = info->mimes[i]->type;
        mime->desc = info->mimes[i]->desc;
        mime->suffixes = info->mimes[i]->suffixes;
        mime->plugin = result;
        result->mimes.append(mime);
    }
    return result;
}

class PluginCache {
public:
    PluginCache() : m_loaded(false), m_refresh(false) {}
    ~PluginCache() { reset(false); }

    void reset(bool refresh)
    {
        for (size_t i = 0; i < m_plugins.size(); ++i)
            deleteAllValues(m_plugins[i]->mimes);
        deleteAllValues(m_plugins);
        m_plugins.clear();
        m_loaded = false;
        m_refresh = refresh;
    }

    const Vector<PluginInfo*>& plugins()
    {
        if (!m_loaded) {
            ChromiumBridge::plugins(m_refresh, &m_plugins);
            m_loaded = true;
            m_refresh = false;
        }
        return m_plugins;
    }

private:
    Vector<PluginInfo*> m_plugins;
    bool m_loaded;
    bool m_refresh;
};

static PluginCache pluginCache;

void PluginData::initPlugins()
{
    const Vector<PluginInfo*>& plugins = pluginCache.plugins();
    for (size_t i = 0; i < plugins.size(); ++i)
        m_plugins.append(clone(plugins[i]));
}

void PluginData::refresh()
{
    pluginCache.reset(true);
    pluginCache.plugins();  // Force the plugins to be reloaded now.
}

String getPluginMimeTypeFromExtension(const String& extension)
{
    const Vector<PluginInfo*>& plugins = pluginCache.plugins();
    for (size_t i = 0; i < plugins.size(); ++i) {
        for (size_t j = 0; j < plugins[i]->mimes.size(); ++j) {
            MimeClassInfo* mime = plugins[i]->mimes[j];
            Vector<String> extensions;
            mime->suffixes.split(",", extensions);
            for (size_t k = 0; k < extensions.size(); ++k) {
                if (extension == extensions[k])
                    return mime->type;
            }
        }
    }
    return String();
}

} // namespace WebCore
