

#include "config.h"
#include "ApplicationCacheHost.h"

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "WebApplicationCacheHostClient.h"
#include "WebKit.h"
#include "WebKitClient.h"

namespace WebCore {

class ApplicationCacheHostInternal : public WebKit::WebApplicationCacheHostClient {
public:
    ApplicationCacheHostInternal(ApplicationCacheHost* host)
        : m_innerHost(host)
    {
        m_outerHost.set(WebKit::webKitClient()->createApplicationCacheHost(this));
    }

    virtual void notifyEventListener(WebKit::WebApplicationCacheHost::EventID eventID)
    {
        m_innerHost->notifyDOMApplicationCache(static_cast<ApplicationCacheHost::EventID>(eventID));
    }

    static WebKit::WebApplicationCacheHost* toWebApplicationCacheHost(ApplicationCacheHost* innerHost)
    {
        if (innerHost && innerHost->m_internal.get())
            return innerHost->m_internal->m_outerHost.get();
        return 0;
    }

private:
    friend class ApplicationCacheHost;
    ApplicationCacheHost* m_innerHost;
    OwnPtr<WebKit::WebApplicationCacheHost> m_outerHost;
};

}

#endif  // ENABLE(OFFLINE_WEB_APPLICATIONS)
