

#include "config.h"
#include "StorageNamespaceProxy.h"

#if ENABLE(DOM_STORAGE)

#include "Chrome.h"
#include "ChromeClientImpl.h"
#include "Page.h"
#include "SecurityOrigin.h"
#include "StorageAreaProxy.h"
#include "WebKit.h"
#include "WebKitClient.h"
#include "WebStorageNamespace.h"
#include "WebString.h"
#include "WebViewClient.h"
#include "WebViewImpl.h"

namespace WebCore {

PassRefPtr<StorageNamespace> StorageNamespace::localStorageNamespace(const String& path, unsigned quota)
{
    return adoptRef(new StorageNamespaceProxy(WebKit::webKitClient()->createLocalStorageNamespace(path, quota), LocalStorage));
}

PassRefPtr<StorageNamespace> StorageNamespace::sessionStorageNamespace(Page* page)
{
    WebKit::ChromeClientImpl* chromeClientImpl = static_cast<WebKit::ChromeClientImpl*>(page->chrome()->client());
    WebKit::WebViewClient* webViewClient = chromeClientImpl->webView()->client();
    return adoptRef(new StorageNamespaceProxy(webViewClient->createSessionStorageNamespace(), SessionStorage));
}

StorageNamespaceProxy::StorageNamespaceProxy(WebKit::WebStorageNamespace* storageNamespace, StorageType storageType)
    : m_storageNamespace(storageNamespace)
    , m_storageType(storageType)
{
}

StorageNamespaceProxy::~StorageNamespaceProxy()
{
}

PassRefPtr<StorageNamespace> StorageNamespaceProxy::copy()
{
    ASSERT(m_storageType == SessionStorage);
    // The WebViewClient knows what its session storage namespace id is but we
    // do not.  Returning 0 here causes it to be fetched (via the WebViewClient)
    // on its next use.  Note that it is WebViewClient::createView's
    // responsibility to clone the session storage namespace id and that the
    // only time copy() is called is directly after the createView call...which
    // is why all of this is safe.
    return 0;
}

PassRefPtr<StorageArea> StorageNamespaceProxy::storageArea(PassRefPtr<SecurityOrigin> origin)
{
    return adoptRef(new StorageAreaProxy(m_storageNamespace->createStorageArea(origin->toString()), m_storageType));
}

void StorageNamespaceProxy::close()
{
    m_storageNamespace->close();
}

void StorageNamespaceProxy::unlock()
{
    // FIXME: Implement.
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
