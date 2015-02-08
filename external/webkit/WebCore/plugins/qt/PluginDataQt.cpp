

#include "config.h"
#include "PluginData.h"

#include "PluginDatabase.h"
#include "PluginPackage.h"

#include "Chrome.h"
#include "ChromeClientQt.h"
#include "Page.h"
#include <qwebpage.h>
#include <qwebpluginfactory.h>

namespace WebCore {

void PluginData::initPlugins()
{
    QWebPage* webPage = static_cast<ChromeClientQt*>(m_page->chrome()->client())->m_webPage;
    QWebPluginFactory* factory = webPage->pluginFactory();
    if (factory) {

        QList<QWebPluginFactory::Plugin> qplugins = factory->plugins();
        for (int i = 0; i < qplugins.count(); ++i) {
            const QWebPluginFactory::Plugin& qplugin = qplugins.at(i);

            PluginInfo* info = new PluginInfo;
            info->name = qplugin.name;
            info->desc = qplugin.description;

            for (int j = 0; j < qplugin.mimeTypes.count(); ++j) {
                const QWebPluginFactory::MimeType& mimeType = qplugin.mimeTypes.at(j);

                MimeClassInfo* mimeInfo = new MimeClassInfo;
                mimeInfo->type = mimeType.name;
                mimeInfo->desc = mimeType.description;
                mimeInfo->suffixes = mimeType.fileExtensions.join(QLatin1String("; "));

                info->mimes.append(mimeInfo);
            }

            m_plugins.append(info);
        }
    }

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
