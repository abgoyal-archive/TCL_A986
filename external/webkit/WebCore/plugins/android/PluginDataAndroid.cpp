

#include "config.h"
#include "PluginData.h"

#include "PluginDatabase.h"
#include "PluginPackage.h"

namespace WebCore {

void PluginData::initPlugins()
{
    PluginDatabase *db = PluginDatabase::installedPlugins();
    const Vector<PluginPackage*> &plugins = db->plugins();

    for (unsigned int i = 0; i < plugins.size(); ++i) {
        PluginInfo* info = new PluginInfo;
        PluginPackage* package = plugins[i];

        info->name = package->name();
        info->file = package->fileName();
        info->desc = package->description();

        const MIMEToDescriptionsMap& mimeToDescriptions = package->mimeToDescriptions();
        MIMEToDescriptionsMap::const_iterator end = mimeToDescriptions.end();
        for (MIMEToDescriptionsMap::const_iterator it = mimeToDescriptions.begin(); it != end; ++it) {
            MimeClassInfo* mime = new MimeClassInfo;
            info->mimes.append(mime);

            mime->type = it->first;
            mime->desc = it->second;
            mime->plugin = info;

            Vector<String> extensions = package->mimeToExtensions().get(mime->type);

            for (unsigned i = 0; i < extensions.size(); i++) {
                if (i > 0)
                    mime->suffixes += ",";

                mime->suffixes += extensions[i];
            }
        }

        m_plugins.append(info);
    }
}

void PluginData::refresh()
{
    PluginDatabase *db = PluginDatabase::installedPlugins();
    db->refresh();
}

};
