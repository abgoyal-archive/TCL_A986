

#ifndef WebStorageNamespaceImpl_h
#define WebStorageNamespaceImpl_h

#if ENABLE(DOM_STORAGE)

#include "StorageNamespaceImpl.h"
#include "WebStorageNamespace.h"

namespace WebKit {

class WebStorageNamespaceImpl : public WebStorageNamespace {
public:
    WebStorageNamespaceImpl(PassRefPtr<WebCore::StorageNamespace> storageNamespace);
    virtual ~WebStorageNamespaceImpl();
    virtual WebStorageArea* createStorageArea(const WebString& origin);
    virtual WebStorageNamespace* copy();
    virtual void close();

private:
    RefPtr<WebCore::StorageNamespace> m_storageNamespace;
};

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)

#endif // WebStorageNamespaceImpl_h
