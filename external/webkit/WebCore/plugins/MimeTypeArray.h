

#ifndef MimeTypeArray_h
#define MimeTypeArray_h

#include "MimeType.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

    class AtomicString;
    class Frame;
    class PluginData;

    class MimeTypeArray : public RefCounted<MimeTypeArray> {
    public:
        static PassRefPtr<MimeTypeArray> create(Frame* frame) { return adoptRef(new MimeTypeArray(frame)); }
        ~MimeTypeArray();

        void disconnectFrame() { m_frame = 0; }

        unsigned length() const;
        PassRefPtr<MimeType> item(unsigned index);
        bool canGetItemsForName(const AtomicString& propertyName);
        PassRefPtr<MimeType> namedItem(const AtomicString& propertyName);

    private:
        MimeTypeArray(Frame*);
        PluginData* getPluginData() const;

        Frame* m_frame;
    };

} // namespace WebCore

#endif // MimeTypeArray_h
