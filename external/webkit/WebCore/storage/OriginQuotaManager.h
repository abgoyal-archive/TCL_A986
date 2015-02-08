

#ifndef OriginQuotaManager_h
#define OriginQuotaManager_h

#if ENABLE(DATABASE)

#include "StringHash.h"
#include "SecurityOriginHash.h"
#include <wtf/HashMap.h>
#include <wtf/Threading.h>

namespace WebCore {

class Database;
class OriginUsageRecord;

class OriginQuotaManager : public Noncopyable {
public:
    OriginQuotaManager();

    void lock();
    void unlock();

    void trackOrigin(PassRefPtr<SecurityOrigin>);
    bool tracksOrigin(SecurityOrigin*) const;
    void addDatabase(SecurityOrigin*, const String& databaseIdentifier, const String& fullPath);
    void removeDatabase(SecurityOrigin*, const String& databaseIdentifier);
    void removeOrigin(SecurityOrigin*);

    void markDatabase(Database*); // Mark dirtiness of a specific database.
    unsigned long long diskUsage(SecurityOrigin*) const;

private:
    mutable Mutex m_usageRecordGuard;
#ifndef NDEBUG
    bool m_usageRecordGuardLocked;
#endif

    typedef HashMap<RefPtr<SecurityOrigin>, OriginUsageRecord*, SecurityOriginHash> OriginUsageMap;
    OriginUsageMap m_usageMap;
};

} // namespace WebCore

#endif // ENABLE(DATABASE)

#endif // OriginQuotaManager_h
