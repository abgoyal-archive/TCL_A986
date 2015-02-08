
#ifndef DedicatedWorkerThread_h
#define DedicatedWorkerThread_h

#if ENABLE(WORKERS)

#include "WorkerThread.h"

namespace WebCore {

    class WorkerObjectProxy;

    class DedicatedWorkerThread : public WorkerThread {
    public:
        static PassRefPtr<DedicatedWorkerThread> create(const KURL& scriptURL, const String& userAgent, const String& sourceCode, WorkerLoaderProxy&, WorkerObjectProxy&);
        WorkerObjectProxy& workerObjectProxy() const { return m_workerObjectProxy; }
        ~DedicatedWorkerThread();

    protected:
        virtual PassRefPtr<WorkerContext> createWorkerContext(const KURL& url, const String& userAgent);
        virtual void runEventLoop();

    private:
        DedicatedWorkerThread(const KURL&, const String& userAgent, const String& sourceCode, WorkerLoaderProxy&, WorkerObjectProxy&);

        WorkerObjectProxy& m_workerObjectProxy;
    };
} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // DedicatedWorkerThread_h
