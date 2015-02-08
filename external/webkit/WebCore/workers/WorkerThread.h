

#ifndef WorkerThread_h
#define WorkerThread_h

#if ENABLE(WORKERS)

#include "WorkerRunLoop.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class KURL;
    class NotificationPresenter;
    class String;
    class WorkerContext;
    class WorkerLoaderProxy;
    class WorkerReportingProxy;
    struct WorkerThreadStartupData;

    class WorkerThread : public RefCounted<WorkerThread> {
    public:
        virtual ~WorkerThread();

        bool start();
        void stop();

        ThreadIdentifier threadID() const { return m_threadID; }
        WorkerRunLoop& runLoop() { return m_runLoop; }
        WorkerLoaderProxy& workerLoaderProxy() const { return m_workerLoaderProxy; }
        WorkerReportingProxy& workerReportingProxy() const { return m_workerReportingProxy; }

        // Number of active worker threads.
        static unsigned workerThreadCount();

#if ENABLE(NOTIFICATIONS)
        NotificationPresenter* getNotificationPresenter() { return m_notificationPresenter; }
        void setNotificationPresenter(NotificationPresenter* presenter) { m_notificationPresenter = presenter; }
#endif

    protected:
        WorkerThread(const KURL&, const String& userAgent, const String& sourceCode, WorkerLoaderProxy&, WorkerReportingProxy&);

        // Factory method for creating a new worker context for the thread.
        virtual PassRefPtr<WorkerContext> createWorkerContext(const KURL& url, const String& userAgent) = 0;

        // Executes the event loop for the worker thread. Derived classes can override to perform actions before/after entering the event loop.
        virtual void runEventLoop();

        WorkerContext* workerContext() { return m_workerContext.get(); }

    private:
        // Static function executed as the core routine on the new thread. Passed a pointer to a WorkerThread object.
        static void* workerThreadStart(void*);
        void* workerThread();

        ThreadIdentifier m_threadID;
        WorkerRunLoop m_runLoop;
        WorkerLoaderProxy& m_workerLoaderProxy;
        WorkerReportingProxy& m_workerReportingProxy;

        RefPtr<WorkerContext> m_workerContext;
        Mutex m_threadCreationMutex;

        OwnPtr<WorkerThreadStartupData> m_startupData;

#if ENABLE(NOTIFICATIONS)
        NotificationPresenter* m_notificationPresenter;
#endif

        // Track the number of WorkerThread instances for use in layout tests.
        static unsigned m_threadCount;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerThread_h

