

#ifndef StorageNamespace_h
#define StorageNamespace_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class Page;
class SecurityOrigin;
class StorageArea;

// This interface is required for Chromium since these actions need to be proxied between processes.
class StorageNamespace : public RefCounted<StorageNamespace> {
public:
    static PassRefPtr<StorageNamespace> localStorageNamespace(const String& path, unsigned quota);
    static PassRefPtr<StorageNamespace> sessionStorageNamespace(Page*);

    virtual ~StorageNamespace() { }
    virtual PassRefPtr<StorageArea> storageArea(PassRefPtr<SecurityOrigin>) = 0;
    virtual PassRefPtr<StorageNamespace> copy() = 0;
    virtual void close() = 0;
    virtual void unlock() = 0;

#ifdef ANDROID
    virtual void clear(Page*) = 0;
#endif
};

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageNamespace_h
