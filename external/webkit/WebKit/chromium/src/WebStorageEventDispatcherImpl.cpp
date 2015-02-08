

#include "config.h"
#include "WebStorageEventDispatcherImpl.h"

#if ENABLE(DOM_STORAGE)

#include "KURL.h"
#include "SecurityOrigin.h"

#include "WebStorageAreaImpl.h"
#include "WebURL.h"

namespace WebKit {

extern const char* pageGroupName;

WebStorageEventDispatcher* WebStorageEventDispatcher::create()
{
    return new WebStorageEventDispatcherImpl();
}

WebStorageEventDispatcherImpl::WebStorageEventDispatcherImpl()
    : m_eventDispatcher(new WebCore::StorageEventDispatcherImpl(pageGroupName))
{
    ASSERT(m_eventDispatcher);
}

void WebStorageEventDispatcherImpl::dispatchStorageEvent(const WebString& key, const WebString& oldValue,
                                                         const WebString& newValue, const WebString& origin,
                                                         const WebURL& passedInURL, bool isLocalStorage)
{
    // Hack for single-process mode and test shell.
    const WebURL* storageAreaImplURL = WebStorageAreaImpl::currentStorageEventURL();
    const WebURL& url = storageAreaImplURL ? *storageAreaImplURL : passedInURL;

    WebCore::StorageType storageType = isLocalStorage ? WebCore::LocalStorage : WebCore::SessionStorage;
    RefPtr<WebCore::SecurityOrigin> securityOrigin = WebCore::SecurityOrigin::createFromString(origin);
    m_eventDispatcher->dispatchStorageEvent(key, oldValue, newValue, securityOrigin.get(), url, storageType);
}

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)
