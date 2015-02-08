

#ifndef PluginDatabase_H
#define PluginDatabase_H

#include "PlatformString.h"
#include "PluginPackage.h"
#include "StringHash.h"

#include <wtf/HashSet.h>
#include <wtf/Vector.h>

#if defined(ANDROID_PLUGINS)
namespace android {
    class WebSettings;
}
#endif

namespace WebCore {
    class Element;
    class Frame;
    class IntSize;
    class KURL;
    class PluginDatabaseClient;
    class PluginPackage;

    typedef HashSet<RefPtr<PluginPackage>, PluginPackageHash> PluginSet;

    class PluginDatabase : public Noncopyable {
    public:
        PluginDatabase();

        // The first call to installedPlugins creates the plugin database
        // and by default populates it with the plugins installed on the system.
        // For testing purposes, it is possible to not populate the database
        // automatically, as the plugins might affect the DRT results by
        // writing to a.o. stderr.
        static PluginDatabase* installedPlugins(bool populate = true);

        bool refresh();
        void clear();
        Vector<PluginPackage*> plugins() const;
        bool isMIMETypeRegistered(const String& mimeType);
        void addExtraPluginDirectory(const String&);

        static bool isPreferredPluginDirectory(const String& directory);
        static int preferredPluginCompare(const void*, const void*);

        PluginPackage* findPlugin(const KURL&, String& mimeType);
        PluginPackage* pluginForMIMEType(const String& mimeType);
        void setPreferredPluginForMIMEType(const String& mimeType, PluginPackage* plugin);

        void setPluginDirectories(const Vector<String>& directories)
        {
            clear();
            m_pluginDirectories = directories;
        }

        void setClient(PluginDatabaseClient* client)
        {
            m_client = client;
        }

        static Vector<String> defaultPluginDirectories();
        Vector<String> pluginDirectories() const { return m_pluginDirectories; }

        String MIMETypeForExtension(const String& extension) const;

    private:
        void getPluginPathsInDirectories(HashSet<String>&) const;
        void getDeletedPlugins(PluginSet&) const;

        // Returns whether the plugin was actually added or not (it won't be added if it's a duplicate of an existing plugin).
        bool add(PassRefPtr<PluginPackage>);
        void remove(PluginPackage*);

        Vector<String> m_pluginDirectories;
        HashSet<String> m_registeredMIMETypes;
        PluginSet m_plugins;
        HashMap<String, RefPtr<PluginPackage> > m_pluginsByPath;
        HashMap<String, time_t> m_pluginPathsWithTimes;

#if defined(ANDROID_PLUGINS)
        // Need access to setPluginDirectories() to change the default
        // path after startup.
        friend class ::android::WebSettings;
#endif
        HashMap<String, RefPtr<PluginPackage> > m_preferredPlugins;
        PluginDatabaseClient* m_client;
    };

} // namespace WebCore

#endif
