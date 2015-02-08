

#ifndef WebSharedWorker_h
#define WebSharedWorker_h

#include "WebCommon.h"

namespace WebCore {
class ScriptExecutionContext;
}

namespace WebKit {

class WebString;
class WebMessagePortChannel;
class WebCommonWorkerClient;
class WebURL;

// This is the interface to a SharedWorker thread.
// Since SharedWorkers communicate entirely through MessagePorts this interface only contains APIs for starting up a SharedWorker.
class WebSharedWorker {
public:
    // Invoked from the worker thread to instantiate a WebSharedWorker that interacts with the WebKit worker components.
    WEBKIT_API static WebSharedWorker* create(WebCommonWorkerClient*);

    virtual ~WebSharedWorker() {};

    // Returns false if the thread hasn't been started yet (script loading has not taken place).
    // FIXME(atwilson): Remove this when we move the initial script loading into the worker process.
    virtual bool isStarted() = 0;

    virtual void startWorkerContext(const WebURL& scriptURL,
                                    const WebString& name,
                                    const WebString& userAgent,
                                    const WebString& sourceCode) = 0;
    class ConnectListener {
    public:
        // Invoked once the connect event has been sent so the caller can free this object.
        virtual void connected() = 0;
    };

    // Sends a connect event to the SharedWorker context. The listener is invoked when this async operation completes.
    virtual void connect(WebMessagePortChannel*, ConnectListener*) = 0;

    // Invoked to shutdown the worker when there are no more associated documents.
    virtual void terminateWorkerContext() = 0;

    // Notification when the WebCommonWorkerClient is destroyed.
    virtual void clientDestroyed() = 0;
};

} // namespace WebKit

#endif
