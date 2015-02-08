

#ifndef WorkerMessagingProxy_h
#define WorkerMessagingProxy_h

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"
#include "WorkerContextProxy.h"
#include "WorkerLoaderProxy.h"
#include "WorkerObjectProxy.h"
#include <wtf/Noncopyable.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class DedicatedWorkerThread;
    class ScriptExecutionContext;
    class String;
    class Worker;

    class WorkerMessagingProxy : public WorkerContextProxy, public WorkerObjectProxy, public WorkerLoaderProxy, public Noncopyable {
    public:
        WorkerMessagingProxy(Worker*);

        // Implementations of WorkerContextProxy.
        // (Only use these methods in the worker object thread.)
        virtual void startWorkerContext(const KURL& scriptURL, const String& userAgent, const String& sourceCode);
        virtual void terminateWorkerContext();
        virtual void postMessageToWorkerContext(PassRefPtr<SerializedScriptValue>, PassOwnPtr<MessagePortChannelArray>);
        virtual bool hasPendingActivity() const;
        virtual void workerObjectDestroyed();

        // Implementations of WorkerObjectProxy.
        // (Only use these methods in the worker context thread.)
        virtual void postMessageToWorkerObject(PassRefPtr<SerializedScriptValue>, PassOwnPtr<MessagePortChannelArray>);
        virtual void postExceptionToWorkerObject(const String& errorMessage, int lineNumber, const String& sourceURL);
        virtual void postConsoleMessageToWorkerObject(MessageDestination, MessageSource, MessageType, MessageLevel, const String& message, int lineNumber, const String& sourceURL);
        virtual void confirmMessageFromWorkerObject(bool hasPendingActivity);
        virtual void reportPendingActivity(bool hasPendingActivity);
        virtual void workerContextClosed();
        virtual void workerContextDestroyed();

        // Implementation of WorkerLoaderProxy.
        // These methods are called on different threads to schedule loading
        // requests and to send callbacks back to WorkerContext.
        virtual void postTaskToLoader(PassOwnPtr<ScriptExecutionContext::Task>);
        virtual void postTaskForModeToWorkerContext(PassOwnPtr<ScriptExecutionContext::Task>, const String& mode);

        void workerThreadCreated(PassRefPtr<DedicatedWorkerThread>);

        // Only use this method on the worker object thread.
        bool askedToTerminate() const { return m_askedToTerminate; }

    private:
        friend class MessageWorkerTask;
        friend class WorkerContextDestroyedTask;
        friend class WorkerExceptionTask;
        friend class WorkerThreadActivityReportTask;

        virtual ~WorkerMessagingProxy();

        void workerContextDestroyedInternal();
        void reportPendingActivityInternal(bool confirmingMessage, bool hasPendingActivity);
        Worker* workerObject() const { return m_workerObject; }

        RefPtr<ScriptExecutionContext> m_scriptExecutionContext;
        Worker* m_workerObject;
        RefPtr<DedicatedWorkerThread> m_workerThread;

        unsigned m_unconfirmedMessageCount; // Unconfirmed messages from worker object to worker thread.
        bool m_workerThreadHadPendingActivity; // The latest confirmation from worker thread reported that it was still active.

        bool m_askedToTerminate;

        Vector<OwnPtr<ScriptExecutionContext::Task> > m_queuedEarlyTasks; // Tasks are queued here until there's a thread object created.
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerMessagingProxy_h
