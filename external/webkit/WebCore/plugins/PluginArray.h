

#ifndef PluginArray_h
#define PluginArray_h

#include "Plugin.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

    class AtomicString;
    class Frame;
    class PluginData;

    class PluginArray : public RefCounted<PluginArray> {
    public:
        static PassRefPtr<PluginArray> create(Frame* frame) { return adoptRef(new PluginArray(frame)); }
        ~PluginArray();

        void disconnectFrame() { m_frame = 0; }

        unsigned length() const;
        PassRefPtr<Plugin> item(unsigned index);
        bool canGetItemsForName(const AtomicString& propertyName);
        PassRefPtr<Plugin> namedItem(const AtomicString& propertyName);

        void refresh(bool reload);

    private:
        PluginArray(Frame*);
        PluginData* getPluginData() const;

        Frame* m_frame;
    };

} // namespace WebCore

#endif // PluginArray_h
