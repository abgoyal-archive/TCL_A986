
#include "config.h"
#include "OriginQuotaManager.h"

#if ENABLE(DATABASE)

#include "Database.h"
#include "OriginUsageRecord.h"

namespace WebCore {

OriginQuotaManager::OriginQuotaManager()
#ifndef NDEBUG
    : m_usageRecordGuardLocked(false)
#endif
{
}

void OriginQuotaManager::lock()
{
    m_usageRecordGuard.lock();
#ifndef NDEBUG
    m_usageRecordGuardLocked = true;
#endif
}

void OriginQuotaManager::unlock()
{
#ifndef NDEBUG
    m_usageRecordGuardLocked = false;
#endif
    m_usageRecordGuard.unlock();
}

void OriginQuotaManager::trackOrigin(PassRefPtr<SecurityOrigin> origin)
{
    ASSERT(m_usageRecordGuardLocked);
    ASSERT(!m_usageMap.contains(origin.get()));

    m_usageMap.set(origin, new OriginUsageRecord);
}

bool OriginQuotaManager::tracksOrigin(SecurityOrigin* origin) const
{
    ASSERT(m_usageRecordGuardLocked);
    return m_usageMap.contains(origin);
}

void OriginQuotaManager::addDatabase(SecurityOrigin* origin, const String& databaseIdentifier, const String& fullPath)
{
    ASSERT(m_usageRecordGuardLocked);

    OriginUsageRecord* usageRecord = m_usageMap.get(origin);
    ASSERT(usageRecord);

    usageRecord->addDatabase(databaseIdentifier.threadsafeCopy(), fullPath.threadsafeCopy());
}

void OriginQuotaManager::removeDatabase(SecurityOrigin* origin, const String& databaseIdentifier)
{
    ASSERT(m_usageRecordGuardLocked);

    if (OriginUsageRecord* usageRecord = m_usageMap.get(origin))
        usageRecord->removeDatabase(databaseIdentifier);
}

void OriginQuotaManager::removeOrigin(SecurityOrigin* origin)
{
    ASSERT(m_usageRecordGuardLocked);

    if (OriginUsageRecord* usageRecord = m_usageMap.get(origin)) {
        m_usageMap.remove(origin);
        delete usageRecord;
    }
}

void OriginQuotaManager::markDatabase(Database* database)
{
    ASSERT(database);
    ASSERT(m_usageRecordGuardLocked);
    OriginUsageRecord* usageRecord = m_usageMap.get(database->securityOrigin());
    ASSERT(usageRecord);

    usageRecord->markDatabase(database->stringIdentifier());
}

unsigned long long OriginQuotaManager::diskUsage(SecurityOrigin* origin) const
{
    ASSERT(m_usageRecordGuardLocked);

    OriginUsageRecord* usageRecord = m_usageMap.get(origin);
    ASSERT(usageRecord);

    return usageRecord->diskUsage();
}

}

#endif // ENABLE(DATABASE)
