

#ifndef WebURLLoader_h
#define WebURLLoader_h

#include "WebCommon.h"

namespace WebKit {

class WebData;
class WebURLLoaderClient;
class WebURLRequest;
class WebURLResponse;
struct WebURLError;

class WebURLLoader {
public:
    // The WebURLLoader may be deleted in a call to its client.
    virtual ~WebURLLoader() {}

    // Load the request synchronously, returning results directly to the
    // caller upon completion.  There is no mechanism to interrupt a
    // synchronous load!!
    virtual void loadSynchronously(const WebURLRequest&,
        WebURLResponse&, WebURLError&, WebData& data) = 0;

    // Load the request asynchronously, sending notifications to the given
    // client.  The client will receive no further notifications if the
    // loader is disposed before it completes its work.
    virtual void loadAsynchronously(const WebURLRequest&,
        WebURLLoaderClient*) = 0;

    // Cancels an asynchronous load.  This will appear as a load error to
    // the client.
    virtual void cancel() = 0;

    // Suspends/resumes an asynchronous load.
    virtual void setDefersLoading(bool) = 0;
};

} // namespace WebKit

#endif
