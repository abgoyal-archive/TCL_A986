
#include "config.h"
#include "OriginUsageRecord.h"

#if ENABLE(DATABASE)

#include "SQLiteFileSystem.h"

namespace WebCore {

OriginUsageRecord::OriginUsageRecord()
    : m_cachedDiskUsageIsValid(false)
{
}

void OriginUsageRecord::addDatabase(const String& identifier, const String& fullPath)
{
    ASSERT(!m_databaseMap.contains(identifier));
    ASSERT_ARG(identifier, identifier.impl()->refCount() == 1);
    ASSERT_ARG(fullPath, fullPath.impl()->refCount() == 1);

    m_databaseMap.set(identifier, DatabaseEntry(fullPath));
    m_unknownSet.add(identifier);

    m_cachedDiskUsageIsValid = false;
}

void OriginUsageRecord::removeDatabase(const String& identifier)
{
    ASSERT(m_databaseMap.contains(identifier));

    m_databaseMap.remove(identifier);
    m_unknownSet.remove(identifier);
    m_cachedDiskUsageIsValid = false;
}

void OriginUsageRecord::markDatabase(const String& identifier)
{
    ASSERT(m_databaseMap.contains(identifier));
    ASSERT_ARG(identifier, identifier.impl()->refCount() == 1);

    m_unknownSet.add(identifier);
    m_cachedDiskUsageIsValid = false;
}

unsigned long long OriginUsageRecord::diskUsage()
{
    // Use the last cached usage value if we have it.
    if (m_cachedDiskUsageIsValid)
        return m_cachedDiskUsage;

    // stat() for the sizes known to be dirty.
    HashSet<String>::iterator iUnknown = m_unknownSet.begin();
    HashSet<String>::iterator endUnknown = m_unknownSet.end();
    for (; iUnknown != endUnknown; ++iUnknown) {
        const String& path = m_databaseMap.get(*iUnknown).filename;
        ASSERT(!path.isEmpty());

        // When we can't determine the file size, we'll just have to assume the file is missing/inaccessible.
        long long size = SQLiteFileSystem::getDatabaseFileSize(path);
        m_databaseMap.set(*iUnknown, DatabaseEntry(path, size));
    }
    m_unknownSet.clear();

    // Recalculate the cached usage value.
    m_cachedDiskUsage = 0;
    HashMap<String, DatabaseEntry>::iterator iDatabase = m_databaseMap.begin();
    HashMap<String, DatabaseEntry>::iterator endDatabase = m_databaseMap.end();
    for (; iDatabase != endDatabase; ++iDatabase)
        m_cachedDiskUsage += iDatabase->second.size;

    m_cachedDiskUsageIsValid = true;
    return m_cachedDiskUsage;
}

}

#endif
