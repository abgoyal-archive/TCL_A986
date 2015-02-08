

#include "config.h"
#include "DocumentLoader.h"
#include "MainResourceLoader.h"
#include "ResourceHandle.h"
#include "ResourceLoader.h"
#include <wtf/UnusedParam.h>

namespace WebCore {

#ifndef BUILDING_ON_TIGER
static void scheduleAll(const ResourceLoaderSet& loaders, SchedulePair* pair)
{
    const ResourceLoaderSet copy = loaders;
    ResourceLoaderSet::const_iterator end = copy.end();
    for (ResourceLoaderSet::const_iterator it = copy.begin(); it != end; ++it)
        if (ResourceHandle* handle = (*it)->handle())
            handle->schedule(pair);
}

static void unscheduleAll(const ResourceLoaderSet& loaders, SchedulePair* pair)
{
    const ResourceLoaderSet copy = loaders;
    ResourceLoaderSet::const_iterator end = copy.end();
    for (ResourceLoaderSet::const_iterator it = copy.begin(); it != end; ++it)
        if (ResourceHandle* handle = (*it)->handle())
            handle->unschedule(pair);
}
#endif

void DocumentLoader::schedule(SchedulePair* pair)
{
#ifndef BUILDING_ON_TIGER
    if (m_mainResourceLoader && m_mainResourceLoader->handle())
        m_mainResourceLoader->handle()->schedule(pair);
    scheduleAll(m_subresourceLoaders, pair);
    scheduleAll(m_plugInStreamLoaders, pair);
    scheduleAll(m_multipartSubresourceLoaders, pair);
#else
    UNUSED_PARAM(pair);
#endif
}

void DocumentLoader::unschedule(SchedulePair* pair)
{
#ifndef BUILDING_ON_TIGER
    if (m_mainResourceLoader && m_mainResourceLoader->handle())
        m_mainResourceLoader->handle()->unschedule(pair);
    unscheduleAll(m_subresourceLoaders, pair);
    unscheduleAll(m_plugInStreamLoaders, pair);
    unscheduleAll(m_multipartSubresourceLoaders, pair);
#else
    UNUSED_PARAM(pair);
#endif
}

} // namespace
