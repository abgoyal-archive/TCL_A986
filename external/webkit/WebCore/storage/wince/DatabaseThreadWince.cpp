

#include "config.h"
#include "DatabaseThread.h"

#include "Database.h"
#include "DatabaseTask.h"

namespace WebCore {

DatabaseThread::DatabaseThread()
: m_timer(this, &DatabaseThread::timerFired)
{
}

DatabaseThread::~DatabaseThread()
{
}

void DatabaseThread::requestTermination()
{
    m_queue.clear();
}

bool DatabaseThread::terminationRequested() const
{
    return m_queue.isEmpty();
}

void DatabaseThread::timerFired(Timer<DatabaseThread>*)
{
    if (!m_queue.isEmpty()) {
        RefPtr<DatabaseTask> task = m_queue.first();
        task->performTask();
        m_queue.removeFirst();
        if (!m_queue.isEmpty())
            m_timer.startOneShot(0);
    }
}

void DatabaseThread::scheduleTask(PassRefPtr<DatabaseTask> task)
{
    m_queue.append(task);
    if (!m_timer.isActive())
        m_timer.startOneShot(0);
}

void DatabaseThread::scheduleImmediateTask(PassRefPtr<DatabaseTask> task)
{
    task->performTask();
}

void DatabaseThread::unscheduleDatabaseTasks(Database* database)
{
    Deque<RefPtr<DatabaseTask> > reservedTasks;
    for (Deque<RefPtr<DatabaseTask> >::const_iterator i = m_queue.begin(); i != m_queue.end(); ++i) {
        if ((*i)->database() != database)
            reservedTasks.append(*i);
    }

    m_queue.swap(reservedTasks);
}

void DatabaseThread::recordDatabaseOpen(Database* database)
{
    notImplemented();
}

void DatabaseThread::recordDatabaseClosed(Database* database)
{
    notImplemented();
}

} // namespace WebCore
