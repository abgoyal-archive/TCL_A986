

#include "config.h"
#include "LocalStorageThread.h"

#if ENABLE(DOM_STORAGE)

#include "LocalStorageTask.h"
#include "StorageAreaSync.h"

namespace WebCore {

PassOwnPtr<LocalStorageThread> LocalStorageThread::create()
{
    return new LocalStorageThread;
}

LocalStorageThread::LocalStorageThread()
    : m_threadID(0)
{
}

LocalStorageThread::~LocalStorageThread()
{
    ASSERT(isMainThread());
    ASSERT(!m_threadID);
}

bool LocalStorageThread::start()
{
    ASSERT(isMainThread());
    if (!m_threadID)
        m_threadID = createThread(LocalStorageThread::threadEntryPointCallback, this, "WebCore: LocalStorage");
    return m_threadID;
}

void* LocalStorageThread::threadEntryPointCallback(void* thread)
{
    return static_cast<LocalStorageThread*>(thread)->threadEntryPoint();
}

void* LocalStorageThread::threadEntryPoint()
{
    ASSERT(!isMainThread());
    while (OwnPtr<LocalStorageTask> task = m_queue.waitForMessage())
        task->performTask();

    return 0;
}

void LocalStorageThread::scheduleTask(PassOwnPtr<LocalStorageTask> task)
{
    ASSERT(isMainThread());
    ASSERT(!m_queue.killed() && m_threadID);
    m_queue.append(task);
}

void LocalStorageThread::terminate()
{
    ASSERT(isMainThread());
    ASSERT(!m_queue.killed() && m_threadID);
    // Even in weird, exceptional cases, don't wait on a nonexistent thread to terminate.
    if (!m_threadID)
        return;

    void* returnValue;
    m_queue.append(LocalStorageTask::createTerminate(this));
    waitForThreadCompletion(m_threadID, &returnValue);
    ASSERT(m_queue.killed());
    m_threadID = 0;
}

void LocalStorageThread::performTerminate()
{
    ASSERT(!isMainThread());
    m_queue.kill();
}

}

#endif // ENABLE(DOM_STORAGE)
