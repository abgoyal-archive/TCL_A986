

#ifndef StorageEvent_h
#define StorageEvent_h

#if ENABLE(DOM_STORAGE)

#include "Event.h"
#include "PlatformString.h"

namespace WebCore {

    class Storage;

    class StorageEvent : public Event {
    public:
        static PassRefPtr<StorageEvent> create();
        static PassRefPtr<StorageEvent> create(const AtomicString& type, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea);

        const String& key() const { return m_key; }
        const String& oldValue() const { return m_oldValue; }
        const String& newValue() const { return m_newValue; }
        const String& uri() const { return m_uri; }
        Storage* storageArea() const { return m_storageArea.get(); }

        void initStorageEvent(const AtomicString& type, bool canBubble, bool cancelable, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea);

        // Needed once we support init<blank>EventNS
        // void initStorageEventNS(in DOMString namespaceURI, in DOMString typeArg, in boolean canBubbleArg, in boolean cancelableArg, in DOMString keyArg, in DOMString oldValueArg, in DOMString newValueArg, in DOMString uriArg, Storage storageAreaArg);

        virtual bool isStorageEvent() const { return true; }

    private:
        StorageEvent();
        StorageEvent(const AtomicString& type, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea);

        String m_key;
        String m_oldValue;
        String m_newValue;
        String m_uri;
        RefPtr<Storage> m_storageArea;
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageEvent_h
