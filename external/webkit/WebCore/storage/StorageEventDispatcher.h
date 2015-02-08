

#ifndef StorageEventDispatcher_h
#define StorageEventDispatcher_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"
#include "StorageArea.h"

namespace WebCore {

    // This is in its own class since Chromium must override it.
    class StorageEventDispatcher {
    public:
        static void dispatch(const String& key, const String& oldValue, const String& newValue, StorageType, SecurityOrigin*, Frame* sourceFrame);

    private:
        // Do not instantiate.
        StorageEventDispatcher();
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageEventDispatcher_h
