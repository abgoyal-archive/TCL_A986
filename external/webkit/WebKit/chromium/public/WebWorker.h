

#ifndef WebWorker_h
#define WebWorker_h

#include "WebMessagePortChannel.h"

namespace WebKit {

class WebString;
class WebURL;
class WebWorkerClient;

// Provides an interface to the script execution context for a worker.
class WebWorker {
public:
    // Instantiates a built-in WebWorker.
    WEBKIT_API static WebWorker* create(WebWorkerClient*);

    virtual ~WebWorker() { }
    virtual void startWorkerContext(const WebURL& scriptURL,
                                    const WebString& userAgent,
                                    const WebString& sourceCode) = 0;
    virtual void terminateWorkerContext() = 0;
    virtual void postMessageToWorkerContext(
        const WebString&,
        const WebMessagePortChannelArray&) = 0;
    virtual void workerObjectDestroyed() = 0;
    virtual void clientDestroyed() = 0;
};

} // namespace WebKit

#endif
