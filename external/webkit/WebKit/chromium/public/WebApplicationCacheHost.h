

#ifndef WebApplicationCacheHost_h
#define WebApplicationCacheHost_h

#include "WebCommon.h"

namespace WebKit {

class WebApplicationCacheHostClient;
class WebURL;
class WebURLRequest;
class WebURLResponse;
struct WebURLError;

// This interface is used by webkit to call out to the embedder. Webkit uses
// the WebKitClient::createApplicationCacheHost method to create instances,
// and calls delete when the instance is no longer needed.
class WebApplicationCacheHost {
public:
    // These values must match WebCore::ApplicationCacheHost::Status values
    enum Status {
        Uncached,
        Idle,
        Checking,
        Downloading,
        UpdateReady,
        Obsolete
    };

    // These values must match WebCore::ApplicationCacheHost::EventID values
    enum EventID {
        CheckingEvent,
        ErrorEvent,
        NoUpdateEvent,
        DownloadingEvent,
        ProgressEvent,
        UpdateReadyEvent,
        CachedEvent,
        ObsoleteEvent
    };

    virtual ~WebApplicationCacheHost() { }

    // Called for every request made within the context.
    virtual void willStartMainResourceRequest(WebURLRequest&) = 0;
    virtual void willStartSubResourceRequest(WebURLRequest&) = 0;

    // One or the other selectCache methods is called after having parsed the <html> tag.
    // The latter returns false if the current document has been identified as a "foreign"
    // entry, in which case the frame navigation will be restarted by webkit.
    virtual void selectCacheWithoutManifest() = 0;
    virtual bool selectCacheWithManifest(const WebURL& manifestURL) = 0;

    // Called as the main resource is retrieved.
    virtual void didReceiveResponseForMainResource(const WebURLResponse&) = 0;
    virtual void didReceiveDataForMainResource(const char* data, int len) = 0;
    virtual void didFinishLoadingMainResource(bool success) = 0;

    // Called on behalf of the scriptable interface.
    virtual Status status() = 0;
    virtual bool startUpdate() = 0;
    virtual bool swapCache() = 0;
};

}  // namespace WebKit

#endif  // WebApplicationCacheHost_h

