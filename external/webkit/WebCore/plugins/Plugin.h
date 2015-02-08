

#ifndef Plugin_h
#define Plugin_h

#include "MimeType.h"
#include <wtf/RefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class AtomicString;
    class Plugin;
    class PluginData;
    class String;

    class Plugin : public RefCounted<Plugin> {
    public:
        static PassRefPtr<Plugin> create(PluginData* pluginData, unsigned index) { return adoptRef(new Plugin(pluginData, index)); }
        ~Plugin();

        String name() const;
        String filename() const;
        String description() const;

        unsigned length() const;

        PassRefPtr<MimeType> item(unsigned index);
        bool canGetItemsForName(const AtomicString& propertyName);
        PassRefPtr<MimeType> namedItem(const AtomicString& propertyName);

    private:
        Plugin(PluginData*, unsigned index);
        RefPtr<PluginData> m_pluginData;
        unsigned m_index;
    };

} // namespace WebCore

#endif // Plugin_h
