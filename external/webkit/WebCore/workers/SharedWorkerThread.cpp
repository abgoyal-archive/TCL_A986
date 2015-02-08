

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "SharedWorkerThread.h"

#include "SharedWorkerContext.h"

namespace WebCore {

PassRefPtr<SharedWorkerThread> SharedWorkerThread::create(const String& name, const KURL& scriptURL, const String& userAgent, const String& sourceCode, WorkerLoaderProxy& workerLoaderProxy, WorkerReportingProxy& workerReportingProxy)
{
    return adoptRef(new SharedWorkerThread(name, scriptURL, userAgent, sourceCode, workerLoaderProxy, workerReportingProxy));
}

SharedWorkerThread::SharedWorkerThread(const String& name, const KURL& url, const String& userAgent, const String& sourceCode, WorkerLoaderProxy& workerLoaderProxy, WorkerReportingProxy& workerReportingProxy)
    : WorkerThread(url, userAgent, sourceCode, workerLoaderProxy, workerReportingProxy)
    , m_name(name.crossThreadString())
{
}

SharedWorkerThread::~SharedWorkerThread()
{
}

PassRefPtr<WorkerContext> SharedWorkerThread::createWorkerContext(const KURL& url, const String& userAgent)
{
    return SharedWorkerContext::create(m_name, url, userAgent, this);
}

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)
