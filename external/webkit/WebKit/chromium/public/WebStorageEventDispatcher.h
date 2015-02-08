

#ifndef WebStorageEventDispatcher_h
#define WebStorageEventDispatcher_h

#include "WebString.h"

namespace WebKit {

class WebURL;

// This is used to dispatch storage events to all pages.
// FIXME: Make this (or something) work for SessionStorage!
class WebStorageEventDispatcher {
public:
    static WebStorageEventDispatcher* create();

    virtual ~WebStorageEventDispatcher() { }

    // Dispatch the actual event.  Doesn't yet work for SessionStorage.
    virtual void dispatchStorageEvent(const WebString& key, const WebString& oldValue,
                                      const WebString& newValue, const WebString& origin,
                                      const WebURL& url, bool isLocalStorage) = 0;
};

} // namespace WebKit

#endif // WebStorageEventDispatcher_h
