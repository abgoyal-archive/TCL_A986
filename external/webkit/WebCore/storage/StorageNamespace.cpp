

#include "config.h"
#include "StorageNamespace.h"

#include "StorageNamespaceImpl.h"

#if PLATFORM(CHROMIUM)
#error "Chromium should not compile this file and instead define its own version of these factories that navigate the multi-process boundry."
#endif

#if ENABLE(DOM_STORAGE)

namespace WebCore {

PassRefPtr<StorageNamespace> StorageNamespace::localStorageNamespace(const String& path, unsigned quota)
{
    return StorageNamespaceImpl::localStorageNamespace(path, quota);
}

// The page argument is only used by the Chromium port.
PassRefPtr<StorageNamespace> StorageNamespace::sessionStorageNamespace(Page*)
{
    return StorageNamespaceImpl::sessionStorageNamespace();
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
