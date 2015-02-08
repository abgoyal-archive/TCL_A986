

#ifndef PluginPackage_H
#define PluginPackage_H

#include "FileSystem.h"
#include "PlatformString.h"
#include "PluginQuirkSet.h"
#include "StringHash.h"
#include "Timer.h"
#include "npruntime_internal.h"
#include <wtf/HashMap.h>
#include <wtf/RefCounted.h>

#if OS(SYMBIAN)
class QPluginLoader;
class NPInterface;
#endif

namespace WebCore {
    typedef HashMap<String, String> MIMEToDescriptionsMap;
    typedef HashMap<String, Vector<String> > MIMEToExtensionsMap;

    class PluginPackage : public RefCounted<PluginPackage> {
    public:
        ~PluginPackage();
        static PassRefPtr<PluginPackage> createPackage(const String& path, const time_t& lastModified);

        const String& name() const { return m_name; }
        const String& description() const { return m_description; }
        const String& path() const { return m_path; }
        const String& fileName() const { return m_fileName; }
        const String& parentDirectory() const { return m_parentDirectory; }
        time_t lastModified() const { return m_lastModified; }

        const MIMEToDescriptionsMap& mimeToDescriptions() const { return m_mimeToDescriptions; }
        const MIMEToExtensionsMap& mimeToExtensions() const { return m_mimeToExtensions; }

        unsigned hash() const;
        static bool equal(const PluginPackage& a, const PluginPackage& b);

        bool load();
        void unload();
        void unloadWithoutShutdown();

        bool isEnabled() const { return m_isEnabled; }
        void setEnabled(bool);

        const NPPluginFuncs* pluginFuncs() const { return &m_pluginFuncs; }
        int compareFileVersion(const PlatformModuleVersion&) const;
        int compare(const PluginPackage&) const;
        PluginQuirkSet quirks() const { return m_quirks; }
        const PlatformModuleVersion& version() const { return m_moduleVersion; }
#if OS(SYMBIAN)
        NPInterface* npInterface() const { return m_npInterface; }
#endif // OS(SYMBIAN)

    private:
        PluginPackage(const String& path, const time_t& lastModified);

#if OS(SYMBIAN)
        NPInterface* m_npInterface;
        QPluginLoader* m_pluginLoader;
#endif // OS(SYMBIAN)
        bool fetchInfo();
        bool isPluginBlacklisted();
        void determineQuirks(const String& mimeType);

        void determineModuleVersionFromDescription();
        void initializeBrowserFuncs();

        bool m_isEnabled;
        bool m_isLoaded;
        int m_loadCount;

        String m_description;
        String m_path;
        String m_fileName;
        String m_name;
        String m_parentDirectory;

        PlatformModuleVersion m_moduleVersion;

        MIMEToDescriptionsMap m_mimeToDescriptions;
        MIMEToExtensionsMap m_mimeToExtensions;

        PlatformModule m_module;
        time_t m_lastModified;

        NPP_ShutdownProcPtr m_NPP_Shutdown;
        NPPluginFuncs m_pluginFuncs;
        NPNetscapeFuncs m_browserFuncs;

        void freeLibrarySoon();
        void freeLibraryTimerFired(Timer<PluginPackage>*);
        Timer<PluginPackage> m_freeLibraryTimer;

        PluginQuirkSet m_quirks;
    };

    struct PluginPackageHash {
        static unsigned hash(const uintptr_t key) { return reinterpret_cast<PluginPackage*>(key)->hash(); }
        static unsigned hash(const RefPtr<PluginPackage>& key) { return key->hash(); }

        static bool equal(const uintptr_t a, const uintptr_t b) { return equal(reinterpret_cast<PluginPackage*>(a), reinterpret_cast<PluginPackage*>(b)); }
        static bool equal(const RefPtr<PluginPackage>& a, const RefPtr<PluginPackage>& b) { return PluginPackage::equal(*a.get(), *b.get()); }
        static const bool safeToCompareToEmptyOrDeleted = false;
    };

} // namespace WebCore

#endif
