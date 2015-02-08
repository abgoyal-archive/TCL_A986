

#ifndef StorageEventDispatcherImpl_h
#define StorageEventDispatcherImpl_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"
#include "StorageArea.h"

namespace WebCore {

class KURL;
class PageGroup;
class SecurityOrigin;

class StorageEventDispatcherImpl {
public:
    StorageEventDispatcherImpl(const String& groupName);

    void dispatchStorageEvent(const String& key, const String& oldValue,
                              const String& newValue, SecurityOrigin*,
                              const KURL&, StorageType);

private:
    PageGroup* m_pageGroup;
};

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageEventDispatcherImpl_h
