

#ifndef WebSharedWorkerImpl_h
#define WebSharedWorkerImpl_h

#include "WebSharedWorker.h"

#if ENABLE(SHARED_WORKERS)

#include "ScriptExecutionContext.h"

#include "WebWorkerBase.h"

namespace WebKit {

// This class is used by the worker process code to talk to the WebCore::SharedWorker implementation.
// It can't use it directly since it uses WebKit types, so this class converts the data types.
// When the WebCore::SharedWorker object wants to call WebCore::WorkerReportingProxy, this class will
// convert to Chrome data types first and then call the supplied WebCommonWorkerClient.
class WebSharedWorkerImpl : public WebWorkerBase, public WebSharedWorker {
public:
    explicit WebSharedWorkerImpl(WebCommonWorkerClient* client);

    // WebSharedWorker methods:
    virtual bool isStarted();
    virtual void startWorkerContext(const WebURL&, const WebString& name, const WebString& userAgent, const WebString& sourceCode);
    virtual void connect(WebMessagePortChannel*, ConnectListener*);
    virtual void terminateWorkerContext();
    virtual void clientDestroyed();

    // WebWorkerBase methods:
    WebWorkerClient* client();
    WebCommonWorkerClient* commonClient() { return m_client; }

private:
    virtual ~WebSharedWorkerImpl();

    static void connectTask(WebCore::ScriptExecutionContext*, WebSharedWorkerImpl*, PassOwnPtr<WebCore::MessagePortChannel>);

    WebCommonWorkerClient* m_client;
};

} // namespace WebKit

#endif // ENABLE(SHARED_WORKERS)

#endif
