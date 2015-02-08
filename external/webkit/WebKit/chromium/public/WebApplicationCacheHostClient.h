

#ifndef WebApplicationCacheHostClient_h
#define WebApplicationCacheHostClient_h

#include "WebApplicationCacheHost.h"
#include "WebCommon.h"

namespace WebKit {

// This interface is used by the embedder to call into webkit.
class WebApplicationCacheHostClient {
public:
    // Called to fire the event in the scriptable interface.
    virtual void notifyEventListener(WebApplicationCacheHost::EventID) = 0;

protected:
    // Should not be deleted by the embedder.
    virtual ~WebApplicationCacheHostClient() { }
};

}  // namespace WebKit

#endif  // WebApplicationCacheHostClient_h

