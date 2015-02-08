

#include "config.h"
#include "webkitprivate.h"

#include "WorkerThread.h"

unsigned int webkit_worker_thread_count(void)
{
#if ENABLE(WORKERS)
    return WebCore::WorkerThread::workerThreadCount();
#else
    return 0;
#endif
}


