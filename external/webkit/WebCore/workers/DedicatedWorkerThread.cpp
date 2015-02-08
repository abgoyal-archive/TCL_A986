

#include "config.h"

#if ENABLE(WORKERS)

#include "DedicatedWorkerThread.h"

#include "DedicatedWorkerContext.h"
#include "WorkerObjectProxy.h"

namespace WebCore {

PassRefPtr<DedicatedWorkerThread> DedicatedWorkerThread::create(const KURL& scriptURL, const String& userAgent, const String& sourceCode, WorkerLoaderProxy& workerLoaderProxy, WorkerObjectProxy& workerObjectProxy)
{
    return adoptRef(new DedicatedWorkerThread(scriptURL, userAgent, sourceCode, workerLoaderProxy, workerObjectProxy));
}

DedicatedWorkerThread::DedicatedWorkerThread(const KURL& url, const String& userAgent, const String& sourceCode, WorkerLoaderProxy& workerLoaderProxy, WorkerObjectProxy& workerObjectProxy)
    : WorkerThread(url, userAgent, sourceCode, workerLoaderProxy, workerObjectProxy)
    , m_workerObjectProxy(workerObjectProxy)
{
}

DedicatedWorkerThread::~DedicatedWorkerThread()
{
}

PassRefPtr<WorkerContext> DedicatedWorkerThread::createWorkerContext(const KURL& url, const String& userAgent)
{
    return DedicatedWorkerContext::create(url, userAgent, this);
}

void DedicatedWorkerThread::runEventLoop()
{
    // Notify the parent object of our current active state before calling the superclass to run the event loop.
    m_workerObjectProxy.reportPendingActivity(workerContext()->hasPendingActivity());
    WorkerThread::runEventLoop();
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
