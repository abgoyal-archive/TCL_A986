

#include "config.h"
#include "QuotaTracker.h"

#include "CString.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

QuotaTracker& QuotaTracker::instance()
{
    DEFINE_STATIC_LOCAL(QuotaTracker, tracker, ());
    return tracker;
}

void QuotaTracker::getDatabaseSizeAndSpaceAvailableToOrigin(
    const String& originIdentifier, const String& databaseName,
    unsigned long long* databaseSize, unsigned long long* spaceAvailable)
{
    MutexLocker lockData(m_dataGuard);
    ASSERT(m_databaseSizes.contains(originIdentifier));
    HashMap<String, SizeMap>::const_iterator it = m_databaseSizes.find(originIdentifier);
    ASSERT(it->second.contains(databaseName));
    *databaseSize = it->second.get(databaseName);

    ASSERT(m_spaceAvailableToOrigins.contains(originIdentifier));
    *spaceAvailable = m_spaceAvailableToOrigins.get(originIdentifier);
}

void QuotaTracker::updateDatabaseSizeAndSpaceAvailableToOrigin(
    const String& originIdentifier, const String& databaseName,
    unsigned long long databaseSize, unsigned long long spaceAvailable)
{
    MutexLocker lockData(m_dataGuard);
    m_spaceAvailableToOrigins.set(originIdentifier, spaceAvailable);
    HashMap<String, SizeMap>::iterator it = m_databaseSizes.add(originIdentifier, SizeMap()).first;
    it->second.set(databaseName, databaseSize);
}

}
