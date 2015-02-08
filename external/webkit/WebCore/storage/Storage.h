

#ifndef Storage_h
#define Storage_h

#if ENABLE(DOM_STORAGE)

#include <wtf/Forward.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class Frame;
    class StorageArea;
    class String;
    typedef int ExceptionCode;

    class Storage : public RefCounted<Storage> {
    public:
        static PassRefPtr<Storage> create(Frame*, PassRefPtr<StorageArea>);
        ~Storage();

        unsigned length() const;
        String key(unsigned index) const;
        String getItem(const String&) const;
        void setItem(const String& key, const String& value, ExceptionCode&);
        void removeItem(const String&);
        void clear();

        bool contains(const String& key) const;

        Frame* frame() { return m_frame; }
        void disconnectFrame() { m_frame = 0; }

    private:
        Storage(Frame*, PassRefPtr<StorageArea>);

        Frame* m_frame;
        RefPtr<StorageArea> m_storageArea;
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // Storage_h
