
#ifndef SharedWorkerThread_h
#define SharedWorkerThread_h

#if ENABLE(SHARED_WORKERS)

#include "WorkerThread.h"

namespace WebCore {

    class SharedWorkerThread : public WorkerThread {
    public:
        static PassRefPtr<SharedWorkerThread> create(const String& name, const KURL&, const String& userAgent, const String& sourceCode, WorkerLoaderProxy&, WorkerReportingProxy&);
        ~SharedWorkerThread();

    protected:
        virtual PassRefPtr<WorkerContext> createWorkerContext(const KURL&, const String&);

    private:
        SharedWorkerThread(const String& name, const KURL&, const String& userAgent, const String& sourceCode, WorkerLoaderProxy&, WorkerReportingProxy&);

        String m_name;
    };
} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // SharedWorkerThread_h
