

#include "config.h"
#include "LocalStorageThread.h"

#include "LocalStorageTask.h"
#include "StorageAreaSync.h"

namespace WebCore {

LocalStorageThread::LocalStorageThread()
: m_timer(this, &LocalStorageThread::timerFired)
{
}

LocalStorageThread::~LocalStorageThread()
{
}

bool LocalStorageThread::start()
{
    return true;
}

void LocalStorageThread::timerFired(Timer<LocalStorageThread>*)
{
    if (!m_queue.isEmpty()) {
        RefPtr<LocalStorageTask> task = m_queue.first();
        task->performTask();
        m_queue.removeFirst();
        if (!m_queue.isEmpty())
            m_timer.startOneShot(0);
    }
}

void LocalStorageThread::scheduleImport(PassRefPtr<StorageAreaSync> area)
{
    m_queue.append(LocalStorageTask::createImport(area));
    if (!m_timer.isActive())
        m_timer.startOneShot(0);
}

void LocalStorageThread::scheduleSync(PassRefPtr<StorageAreaSync> area)
{
    m_queue.append(LocalStorageTask::createSync(area));
    if (!m_timer.isActive())
        m_timer.startOneShot(0);
}

void LocalStorageThread::terminate()
{
    m_queue.clear();
    m_timer.stop();
}

void LocalStorageThread::performTerminate()
{
    m_queue.clear();
    m_timer.stop();
}

}
