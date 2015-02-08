

#ifndef MimeType_h
#define MimeType_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/RefCounted.h>

#include "PluginData.h"

namespace WebCore {

    class Plugin;
    class String;

    class MimeType : public RefCounted<MimeType> {
    public:
        static PassRefPtr<MimeType> create(PassRefPtr<PluginData> pluginData, unsigned index) { return adoptRef(new MimeType(pluginData, index)); }
        ~MimeType();

        const String &type() const;
        const String &suffixes() const;
        const String &description() const;
        PassRefPtr<Plugin> enabledPlugin() const;

    private:
        MimeType(PassRefPtr<PluginData>, unsigned index);
        RefPtr<PluginData> m_pluginData;
        unsigned m_index;
    };

}

#endif
