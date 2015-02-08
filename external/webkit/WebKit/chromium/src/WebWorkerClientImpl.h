

#ifndef WebWorkerClientImpl_h
#define WebWorkerClientImpl_h

#if ENABLE(WORKERS)

// FIXME: fix to just "WebWorkerClient.h" once nobody in glue depends on us.
#include "../public/WebWorkerClient.h"

#include "WorkerContextProxy.h"
#include <wtf/PassOwnPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
class ScriptExecutionContext;
}

namespace WebKit {
class WebWorker;

// The purpose of this class is to provide a WorkerContextProxy
// implementation that we can give to WebKit.  Internally, it converts the
// data types to Chrome compatible ones so that renderer code can use it over
// IPC.
class WebWorkerClientImpl : public WebCore::WorkerContextProxy
                          , public WebWorkerClient {
public:
    WebWorkerClientImpl(WebCore::Worker*);

    // WebCore::WorkerContextProxy Factory.
    static WebCore::WorkerContextProxy* createWorkerContextProxy(WebCore::Worker*);
    void setWebWorker(WebWorker*);

    // WebCore::WorkerContextProxy methods:
    // These are called on the thread that created the worker.  In the renderer
    // process, this will be the main WebKit thread.  In the worker process, this
    // will be the thread of the executing worker (not the main WebKit thread).
    virtual void startWorkerContext(const WebCore::KURL&,
                                    const WebCore::String&,
                                    const WebCore::String&);
    virtual void terminateWorkerContext();
    virtual void postMessageToWorkerContext(
        PassRefPtr<WebCore::SerializedScriptValue> message,
        PassOwnPtr<WebCore::MessagePortChannelArray> channels);
    virtual bool hasPendingActivity() const;
    virtual void workerObjectDestroyed();

    // WebWorkerClient methods:
    // These are called on the main WebKit thread.
    virtual void postMessageToWorkerObject(const WebString&, const WebMessagePortChannelArray&);
    virtual void postExceptionToWorkerObject(const WebString&, int, const WebString&);
    virtual void postConsoleMessageToWorkerObject(int, int, int, int, const WebString&,
                                                  int, const WebString&);
    virtual void confirmMessageFromWorkerObject(bool);
    virtual void reportPendingActivity(bool);
    virtual void workerContextClosed();
    virtual void workerContextDestroyed();
    virtual WebWorker* createWorker(WebWorkerClient*) { return 0; }
    virtual WebNotificationPresenter* notificationPresenter()
    {
        // FIXME: Notifications not yet supported in workers.
        return 0;
    }

private:
    virtual ~WebWorkerClientImpl();

    // Methods used to support WebWorkerClientImpl being constructed on worker
    // threads.
    // These tasks are dispatched on the WebKit thread.
    static void startWorkerContextTask(WebCore::ScriptExecutionContext* context,
                                       WebWorkerClientImpl* thisPtr,
                                       const WebCore::String& scriptURL,
                                       const WebCore::String& userAgent,
                                       const WebCore::String& sourceCode);
    static void terminateWorkerContextTask(WebCore::ScriptExecutionContext* context,
                                           WebWorkerClientImpl* thisPtr);
    static void postMessageToWorkerContextTask(WebCore::ScriptExecutionContext* context,
                                               WebWorkerClientImpl* thisPtr,
                                               const WebCore::String& message,
                                               PassOwnPtr<WebCore::MessagePortChannelArray> channels);
    static void workerObjectDestroyedTask(WebCore::ScriptExecutionContext* context,
                                          WebWorkerClientImpl* thisPtr);

    // These tasks are dispatched on the thread that created the worker (i.e.
    // main WebKit thread in renderer process, and the worker thread in the
    // worker process).
    static void postMessageToWorkerObjectTask(WebCore::ScriptExecutionContext* context,
                                              WebWorkerClientImpl* thisPtr,
                                              const WebCore::String& message,
                                              PassOwnPtr<WebCore::MessagePortChannelArray> channels);
    static void postExceptionToWorkerObjectTask(WebCore::ScriptExecutionContext* context,
                                                WebWorkerClientImpl* thisPtr,
                                                const WebCore::String& message,
                                                int lineNumber,
                                                const WebCore::String& sourceURL);
    static void postConsoleMessageToWorkerObjectTask(WebCore::ScriptExecutionContext* context,
                                                     WebWorkerClientImpl* thisPtr,
                                                     int destinationId,
                                                     int sourceId,
                                                     int messageType,
                                                     int messageLevel,
                                                     const WebCore::String& message,
                                                     int lineNumber,
                                                     const WebCore::String& sourceURL);
    static void confirmMessageFromWorkerObjectTask(WebCore::ScriptExecutionContext* context,
                                                   WebWorkerClientImpl* thisPtr);
    static void reportPendingActivityTask(WebCore::ScriptExecutionContext* context,
                                          WebWorkerClientImpl* thisPtr,
                                          bool hasPendingActivity);

    // Guard against context from being destroyed before a worker exits.
    RefPtr<WebCore::ScriptExecutionContext> m_scriptExecutionContext;

    WebCore::Worker* m_worker;
    WebWorker* m_webWorker;
    bool m_askedToTerminate;
    unsigned m_unconfirmedMessageCount;
    bool m_workerContextHadPendingActivity;
    ThreadIdentifier m_workerThreadId;
};

} // namespace WebKit;

#endif // ENABLE(WORKERS)

#endif
