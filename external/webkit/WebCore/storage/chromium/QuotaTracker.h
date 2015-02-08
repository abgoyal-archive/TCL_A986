

#ifndef QuotaTracker_h
#define QuotaTracker_h

#include "CString.h"
#include "SecurityOrigin.h"
#include "StringHash.h"
#include <wtf/HashMap.h>

namespace WebCore {

class QuotaTracker {
public:
    static QuotaTracker& instance();

    void getDatabaseSizeAndSpaceAvailableToOrigin(
        const String& originIdentifier, const String& databaseName,
        unsigned long long* databaseSize, unsigned long long* spaceAvailable);
    void updateDatabaseSizeAndSpaceAvailableToOrigin(
        const String& originIdentifier, const String& databaseName,
        unsigned long long databaseSize, unsigned long long spaceAvailable);

private:
    QuotaTracker() { }

    typedef HashMap<String, unsigned long long> SizeMap;
    SizeMap m_spaceAvailableToOrigins;
    HashMap<String, SizeMap> m_databaseSizes;
    Mutex m_dataGuard;
};

}

#endif // QuotaTracker_h
