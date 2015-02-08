

#ifndef WebStorageEventDispatcherImpl_h
#define WebStorageEventDispatcherImpl_h

#if ENABLE(DOM_STORAGE)

#include "StorageEventDispatcherImpl.h"
#include "WebStorageEventDispatcher.h"
#include <wtf/OwnPtr.h>

namespace WebKit {

class WebStorageEventDispatcherImpl : public WebStorageEventDispatcher {
public:
    WebStorageEventDispatcherImpl();

    virtual void dispatchStorageEvent(const WebString& key, const WebString& oldValue,
                                      const WebString& newValue, const WebString& origin,
                                      const WebURL&, bool isLocalStorage);

private:
    OwnPtr<WebCore::StorageEventDispatcherImpl> m_eventDispatcher;
};

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)

#endif // WebStorageEventDispatcherImpl_h
