

#include "config.h"
#include "webkitprivate.h"

#include <wtf/UnusedParam.h>

#include "ApplicationCacheStorage.h"

void webkit_application_cache_set_maximum_size(unsigned long long size)
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)   
    WebCore::cacheStorage().empty();
    WebCore::cacheStorage().vacuumDatabaseFile();
    WebCore::cacheStorage().setMaximumSize(size);
#else
    UNUSED_PARAM(size);
#endif
}
