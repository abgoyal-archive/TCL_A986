

#include "config.h"
#include "StorageEventDispatcher.h"

#if ENABLE(DOM_STORAGE)

#include "SecurityOrigin.h"
#include "StorageArea.h"

#include "WebKit.h"
#include "WebKitClient.h"
#include "WebString.h"
#include "WebURL.h"

namespace WebCore {

void StorageEventDispatcher::dispatch(const String& key, const String& oldValue,
                                      const String& newValue, StorageType storageType,
                                      SecurityOrigin* origin, Frame* sourceFrame)
{
    ASSERT(!sourceFrame);  // Sad, but true.
    WebKit::webKitClient()->dispatchStorageEvent(key, oldValue, newValue, origin->toString(), WebKit::WebURL(), storageType == LocalStorage);
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
