

#ifndef PluginData_h
#define PluginData_h

#include <wtf/RefCounted.h>
#include <wtf/Vector.h>
#include "PlatformString.h"

namespace WebCore {

    class Page;
    struct PluginInfo;

    struct MimeClassInfo : Noncopyable {
        String type;
        String desc;
        String suffixes;
        PluginInfo* plugin;
    };

    struct PluginInfo : Noncopyable {
        String name;
        String file;
        String desc;
        Vector<MimeClassInfo*> mimes;
    };

    // FIXME: merge with PluginDatabase in the future
    class PluginData : public RefCounted<PluginData> {
    public:
        static PassRefPtr<PluginData> create(const Page* page) { return adoptRef(new PluginData(page)); }
        ~PluginData();

        void disconnectPage() { m_page = 0; }
        const Page* page() const { return m_page; }

        const Vector<PluginInfo*>& plugins() const { return m_plugins; }
        const Vector<MimeClassInfo*>& mimes() const { return m_mimes; }

        bool supportsMimeType(const String& mimeType) const;
        String pluginNameForMimeType(const String& mimeType) const;

        static void refresh();

    private:
        PluginData(const Page*);
        void initPlugins();

        Vector<PluginInfo*> m_plugins;
        Vector<MimeClassInfo*> m_mimes;
        const Page* m_page;
    };

}

#endif
