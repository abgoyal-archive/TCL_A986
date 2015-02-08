

#ifndef StorageNamespaceProxy_h
#define StorageNamespaceProxy_h

#if ENABLE(DOM_STORAGE)

#include "StorageArea.h"
#include "StorageNamespace.h"

namespace WebKit { class WebStorageNamespace; }

namespace WebCore {

class StorageNamespaceProxy : public StorageNamespace {
public:
    StorageNamespaceProxy(WebKit::WebStorageNamespace*, StorageType);
    virtual ~StorageNamespaceProxy();
    virtual PassRefPtr<StorageArea> storageArea(PassRefPtr<SecurityOrigin>);
    virtual PassRefPtr<StorageNamespace> copy();
    virtual void close();
    virtual void unlock();

private:
    OwnPtr<WebKit::WebStorageNamespace> m_storageNamespace;
    StorageType m_storageType;
};

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageNamespaceProxy_h
