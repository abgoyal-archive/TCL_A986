

#include "config.h"
#include "StorageSyncManager.h"

#if ENABLE(DOM_STORAGE)

#include "CString.h"
#include "EventNames.h"
#include "FileSystem.h"
#include "Frame.h"
#include "FrameTree.h"
#include "LocalStorageTask.h"
#include "LocalStorageThread.h"
#include "Page.h"
#include "PageGroup.h"
#include "StorageAreaSync.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

PassRefPtr<StorageSyncManager> StorageSyncManager::create(const String& path)
{
    return adoptRef(new StorageSyncManager(path));
}

StorageSyncManager::StorageSyncManager(const String& path)
    : m_thread(LocalStorageThread::create())
    , m_path(path.crossThreadString())
{
    ASSERT(isMainThread());
    ASSERT(!m_path.isEmpty());
    m_thread->start();
}

StorageSyncManager::~StorageSyncManager()
{
    ASSERT(isMainThread());
    ASSERT(!m_thread);
}

// Called on a background thread.
String StorageSyncManager::fullDatabaseFilename(const String& databaseIdentifier)
{
    if (!makeAllDirectories(m_path)) {
        LOG_ERROR("Unabled to create LocalStorage database path %s", m_path.utf8().data());
        return String();
    }

    return pathByAppendingComponent(m_path, databaseIdentifier + ".localstorage");
}

void StorageSyncManager::close()
{
    ASSERT(isMainThread());

    if (m_thread) {
        m_thread->terminate();
        m_thread = 0;
    }
}

bool StorageSyncManager::scheduleImport(PassRefPtr<StorageAreaSync> area)
{
    ASSERT(isMainThread());
    ASSERT(m_thread);
    if (m_thread)
        m_thread->scheduleTask(LocalStorageTask::createImport(area.get()));
    return m_thread;
}

void StorageSyncManager::scheduleSync(PassRefPtr<StorageAreaSync> area)
{
    ASSERT(isMainThread());
    ASSERT(m_thread);
    if (m_thread)
        m_thread->scheduleTask(LocalStorageTask::createSync(area.get()));
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
