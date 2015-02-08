

#include "config.h"
#include "ApplicationCacheResource.h"
#include <stdio.h>

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

namespace WebCore {

ApplicationCacheResource::ApplicationCacheResource(const KURL& url, const ResourceResponse& response, unsigned type, PassRefPtr<SharedBuffer> data)
    : SubstituteResource(url, response, data)
    , m_type(type)
    , m_storageID(0)
    , m_estimatedSizeInStorage(0)
{
}

void ApplicationCacheResource::addType(unsigned type) 
{
    // Caller should take care of storing the new type in database.
    m_type |= type; 
}

int64_t ApplicationCacheResource::estimatedSizeInStorage()
{
    if (m_estimatedSizeInStorage)
      return m_estimatedSizeInStorage;

    if (data())
        m_estimatedSizeInStorage = data()->size();

    HTTPHeaderMap::const_iterator end = response().httpHeaderFields().end();
    for (HTTPHeaderMap::const_iterator it = response().httpHeaderFields().begin(); it != end; ++it)
        m_estimatedSizeInStorage += (it->first.length() + it->second.length() + 2) * sizeof(UChar);

    m_estimatedSizeInStorage += url().string().length() * sizeof(UChar);
    m_estimatedSizeInStorage += sizeof(int); // response().m_httpStatusCode
    m_estimatedSizeInStorage += response().url().string().length() * sizeof(UChar);
    m_estimatedSizeInStorage += sizeof(unsigned); // dataId
    m_estimatedSizeInStorage += response().mimeType().length() * sizeof(UChar);
    m_estimatedSizeInStorage += response().textEncodingName().length() * sizeof(UChar);

    return m_estimatedSizeInStorage;
}

#ifndef NDEBUG
void ApplicationCacheResource::dumpType(unsigned type)
{
    if (type & Master)
        printf("master ");
    if (type & Manifest)
        printf("manifest ");
    if (type & Explicit)
        printf("explicit ");
    if (type & Foreign)
        printf("foreign ");
    if (type & Fallback)
        printf("fallback ");
    
    printf("\n");
}
#endif

} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)
