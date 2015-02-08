

#include "config.h"
#include "LocalStorageTask.h"

#if ENABLE(DOM_STORAGE)

#include "LocalStorageThread.h"
#include "StorageAreaSync.h"

namespace WebCore {

LocalStorageTask::LocalStorageTask(Type type, StorageAreaSync* area)
    : m_type(type)
    , m_area(area)
    , m_thread(0)
{
    ASSERT(m_area);
    ASSERT(m_type == AreaImport || m_type == AreaSync);
}

LocalStorageTask::LocalStorageTask(Type type, LocalStorageThread* thread)
    : m_type(type)
    , m_area(0)
    , m_thread(thread)
{
    ASSERT(m_thread);
    ASSERT(m_type == TerminateThread);
}

LocalStorageTask::~LocalStorageTask()
{
}

void LocalStorageTask::performTask()
{
    switch (m_type) {
        case AreaImport:
            m_area->performImport();
            break;
        case AreaSync:
            m_area->performSync();
            break;
        case TerminateThread:
            m_thread->performTerminate();
            break;
    }
}

}

#endif // ENABLE(DOM_STORAGE)
