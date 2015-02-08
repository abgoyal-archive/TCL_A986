

#ifndef WebCommonWorkerClient_h
#define WebCommonWorkerClient_h

namespace WebKit {

class WebNotificationPresenter;
class WebString;
class WebWorker;
class WebWorkerClient;

// Provides an interface back to the in-page script object for a worker.
// This interface contains common APIs used by both shared and dedicated
// workers.
// All functions are expected to be called back on the thread that created
// the Worker object, unless noted.
class WebCommonWorkerClient {
public:
    virtual void postExceptionToWorkerObject(
        const WebString& errorString, int lineNumber,
        const WebString& sourceURL) = 0;

    virtual void postConsoleMessageToWorkerObject(
        int destinationIdentifier,
        int sourceIdentifier,
        int messageType,
        int messageLevel,
        const WebString& message,
        int lineNumber,
        const WebString& sourceURL) = 0;

    virtual void workerContextClosed() = 0;
    virtual void workerContextDestroyed() = 0;

    // Returns the notification presenter for this worker context.  Pointer
    // is owned by the object implementing WebCommonWorkerClient.
    virtual WebNotificationPresenter* notificationPresenter() = 0;

    // This can be called on any thread to create a nested WebWorker.
    // WebSharedWorkers are not instantiated via this API - instead
    // they are created via the WebSharedWorkerRepository.
    virtual WebWorker* createWorker(WebWorkerClient* client) = 0;

protected:
    ~WebCommonWorkerClient() { }
};

} // namespace WebKit

#endif
