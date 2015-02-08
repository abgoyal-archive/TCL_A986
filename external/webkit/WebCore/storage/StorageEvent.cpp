

#include "config.h"
#include "StorageEvent.h"

#if ENABLE(DOM_STORAGE)

#include "Storage.h"

namespace WebCore {

PassRefPtr<StorageEvent> StorageEvent::create()
{
    return adoptRef(new StorageEvent);
}

StorageEvent::StorageEvent()
{
}

PassRefPtr<StorageEvent> StorageEvent::create(const AtomicString& type, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea)
{
    return adoptRef(new StorageEvent(type, key, oldValue, newValue, uri, storageArea));
}

StorageEvent::StorageEvent(const AtomicString& type, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea)
    : Event(type, false, false)
    , m_key(key)
    , m_oldValue(oldValue)
    , m_newValue(newValue)
    , m_uri(uri)
    , m_storageArea(storageArea)
{
}

void StorageEvent::initStorageEvent(const AtomicString& type, bool canBubble, bool cancelable, const String& key, const String& oldValue, const String& newValue, const String& uri, Storage* storageArea)
{
    if (dispatched())
        return;

    initEvent(type, canBubble, cancelable);

    m_key = key;
    m_oldValue = oldValue;
    m_newValue = newValue;
    m_uri = uri;
    m_storageArea = storageArea;
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
