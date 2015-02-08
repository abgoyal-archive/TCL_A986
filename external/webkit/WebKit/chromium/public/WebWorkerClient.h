

#ifndef WebWorkerClient_h
#define WebWorkerClient_h

#include "WebCommonWorkerClient.h"
#include "WebMessagePortChannel.h"

namespace WebKit {

class WebNotificationPresenter;
class WebString;
class WebWorker;

// Provides an interface back to the in-page script object for a worker.
// All functions are expected to be called back on the thread that created
// the Worker object, unless noted.
class WebWorkerClient : public WebCommonWorkerClient {
public:
    virtual void postMessageToWorkerObject(
        const WebString&,
        const WebMessagePortChannelArray&) = 0;

    virtual void confirmMessageFromWorkerObject(bool hasPendingActivity) = 0;
    virtual void reportPendingActivity(bool hasPendingActivity) = 0;

protected:
    ~WebWorkerClient() { }
};

} // namespace WebKit

#endif
