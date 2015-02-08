

#ifndef WebWorkerImpl_h
#define WebWorkerImpl_h

#include "WebWorker.h"

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"

#include "WebWorkerBase.h"

namespace WebKit {
class WebView;

// This class is used by the worker process code to talk to the WebCore::Worker
// implementation.  It can't use it directly since it uses WebKit types, so this
// class converts the data types.  When the WebCore::Worker object wants to call
// WebCore::WorkerObjectProxy, this class will conver to Chrome data types first
// and then call the supplied WebWorkerClient.
class WebWorkerImpl : public WebWorkerBase, public WebWorker {
public:
    explicit WebWorkerImpl(WebWorkerClient* client);

    // WebWorker methods:
    virtual void startWorkerContext(const WebURL&, const WebString&, const WebString&);
    virtual void terminateWorkerContext();
    virtual void postMessageToWorkerContext(const WebString&, const WebMessagePortChannelArray&);
    virtual void workerObjectDestroyed();
    virtual void clientDestroyed();

    // WebWorkerBase methods:
    virtual WebWorkerClient* client() { return m_client; }
    virtual WebCommonWorkerClient* commonClient();

private:
    virtual ~WebWorkerImpl();

    // Tasks that are run on the worker thread.
    static void postMessageToWorkerContextTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerImpl* thisPtr,
        const WebCore::String& message,
        PassOwnPtr<WebCore::MessagePortChannelArray> channels);

    WebWorkerClient* m_client;

};

} // namespace WebKit

#endif // ENABLE(WORKERS)

#endif
