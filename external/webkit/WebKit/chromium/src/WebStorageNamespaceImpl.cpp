

#include "config.h"
#include "WebStorageNamespaceImpl.h"

#if ENABLE(DOM_STORAGE)

#include "SecurityOrigin.h"

#include "WebStorageAreaImpl.h"
#include "WebString.h"

namespace WebKit {

WebStorageNamespace* WebStorageNamespace::createLocalStorageNamespace(const WebString& path, unsigned quota)
{
    return new WebStorageNamespaceImpl(WebCore::StorageNamespaceImpl::localStorageNamespace(path, quota));
}

WebStorageNamespace* WebStorageNamespace::createSessionStorageNamespace()
{
    return new WebStorageNamespaceImpl(WebCore::StorageNamespaceImpl::sessionStorageNamespace());
}

WebStorageNamespaceImpl::WebStorageNamespaceImpl(PassRefPtr<WebCore::StorageNamespace> storageNamespace)
    : m_storageNamespace(storageNamespace)
{
}

WebStorageNamespaceImpl::~WebStorageNamespaceImpl()
{
}

WebStorageArea* WebStorageNamespaceImpl::createStorageArea(const WebString& originString)
{
    WebCore::String originWebCoreString = originString;
    if (originWebCoreString == "file://") {
        // FIXME: We should really be passing around WebSecurityOrigin objects
        //        to represent security origins instead of strings.  One issue
        //        with using strings is that createFromString(toString) does
        //        not round-trip for file URLs because file:// looks like a
        //        directory (which is sandboxed).
        //
        // For the time being, we work around this issue by using "file:///a",
        // which does not look like a directory.  We should fix this when
        // jorlow gets back from vactation.
        originWebCoreString = "file:///a";
    }
    RefPtr<WebCore::SecurityOrigin> origin = WebCore::SecurityOrigin::createFromString(originWebCoreString);
    return new WebStorageAreaImpl(m_storageNamespace->storageArea(origin.release()));
}

WebStorageNamespace* WebStorageNamespaceImpl::copy()
{
    return new WebStorageNamespaceImpl(m_storageNamespace->copy());
}

void WebStorageNamespaceImpl::close()
{
    m_storageNamespace->close();
}

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)
