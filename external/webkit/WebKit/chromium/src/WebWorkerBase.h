

#ifndef WebWorkerBase_h
#define WebWorkerBase_h

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"
#include "WorkerLoaderProxy.h"
#include "WorkerObjectProxy.h"
#include <wtf/PassOwnPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
class WorkerThread;
}

namespace WebKit {
class WebCommonWorkerClient;
class WebURL;
class WebView;
class WebWorkerClient;

// Base class for WebSharedWorkerImpl and WebWorkerImpl. It contains common
// code used by both implementation classes, including implementations of the
// WorkerObjectProxy and WorkerLoaderProxy interfaces.
class WebWorkerBase : public WebCore::WorkerObjectProxy
                    , public WebCore::WorkerLoaderProxy {
public:
    WebWorkerBase();
    virtual ~WebWorkerBase();

    // WebCore::WorkerObjectProxy methods:
    virtual void postMessageToWorkerObject(
        PassRefPtr<WebCore::SerializedScriptValue>,
        PassOwnPtr<WebCore::MessagePortChannelArray>);
    virtual void postExceptionToWorkerObject(
        const WebCore::String&, int, const WebCore::String&);
    virtual void postConsoleMessageToWorkerObject(
        WebCore::MessageDestination, WebCore::MessageSource, WebCore::MessageType,
        WebCore::MessageLevel, const WebCore::String&, int, const WebCore::String&);
    virtual void confirmMessageFromWorkerObject(bool);
    virtual void reportPendingActivity(bool);
    virtual void workerContextClosed();
    virtual void workerContextDestroyed();

    // WebCore::WorkerLoaderProxy methods:
    virtual void postTaskToLoader(PassOwnPtr<WebCore::ScriptExecutionContext::Task>);
    virtual void postTaskForModeToWorkerContext(
        PassOwnPtr<WebCore::ScriptExecutionContext::Task>, const WebCore::String& mode);

    // Executes the given task on the main thread.
    static void dispatchTaskToMainThread(PassOwnPtr<WebCore::ScriptExecutionContext::Task>);

protected:
    virtual WebWorkerClient* client() = 0;
    virtual WebCommonWorkerClient* commonClient() = 0;

    void setWorkerThread(PassRefPtr<WebCore::WorkerThread> thread) { m_workerThread = thread; }
    WebCore::WorkerThread* workerThread() { return m_workerThread.get(); }

    // Shuts down the worker thread.
    void stopWorkerThread();

    // Creates the shadow loader used for worker network requests.
    void initializeLoader(const WebURL&);

private:
    // Function used to invoke tasks on the main thread.
    static void invokeTaskMethod(void*);

    // Tasks that are run on the main thread.
    static void postMessageTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr,
        WebCore::String message,
        PassOwnPtr<WebCore::MessagePortChannelArray> channels);
    static void postExceptionTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr,
        const WebCore::String& message,
        int lineNumber,
        const WebCore::String& sourceURL);
    static void postConsoleMessageTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr,
        int destination,
        int source,
        int type,
        int level,
        const WebCore::String& message,
        int lineNumber,
        const WebCore::String& sourceURL);
    static void confirmMessageTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr,
        bool hasPendingActivity);
    static void reportPendingActivityTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr,
        bool hasPendingActivity);
    static void workerContextClosedTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr);
    static void workerContextDestroyedTask(
        WebCore::ScriptExecutionContext* context,
        WebWorkerBase* thisPtr);

    // 'shadow page' - created to proxy loading requests from the worker.
    RefPtr<WebCore::ScriptExecutionContext> m_loadingDocument;
    WebView* m_webView;
    bool m_askedToTerminate;

    RefPtr<WebCore::WorkerThread> m_workerThread;
};

} // namespace WebKit

#endif // ENABLE(WORKERS)

#endif
