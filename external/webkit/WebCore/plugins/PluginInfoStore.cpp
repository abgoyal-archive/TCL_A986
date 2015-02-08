

#include "config.h"
#include "PluginInfoStore.h"

#include "KURL.h"
#include "Page.h"
#include "PluginData.h"
#include "PluginDatabase.h"
#include "PluginPackage.h"

namespace WebCore {

PluginInfo* PluginInfoStore::createPluginInfoForPluginAtIndex(unsigned i) 
{ 
    PluginDatabase *db = PluginDatabase::installedPlugins();
    PluginInfo* info = new PluginInfo;
    PluginPackage* package = db->plugins()[i];

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

    return info;
}

unsigned PluginInfoStore::pluginCount() const
{
    return PluginDatabase::installedPlugins()->plugins().size();
}


String PluginInfoStore::pluginNameForMIMEType(const String& mimeType)
{
    String mimeTypeCopy(mimeType);

    if (PluginPackage* package = PluginDatabase::installedPlugins()->findPlugin(KURL(), mimeTypeCopy)) {
        ASSERT(mimeType == mimeTypeCopy);

        return package->name();
    }

    return String();
}

    
bool PluginInfoStore::supportsMIMEType(const WebCore::String& mimeType) 
{
    return PluginDatabase::installedPlugins()->isMIMETypeRegistered(mimeType);
}

void refreshPlugins(bool reloadOpenPages)
{
    Page::refreshPlugins(reloadOpenPages);
}

}
