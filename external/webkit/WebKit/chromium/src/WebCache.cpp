

#include "config.h"
#include "WebCache.h"

// Instead of providing accessors, we make all members of Cache public.
// This will make it easier to track WebCore changes to the Cache class.
// FIXME: We should introduce public getters on the Cache class.
#define private public
#include "Cache.h"
#undef private

using namespace WebCore;

namespace WebKit {

// A helper method for coverting a Cache::TypeStatistic to a
// WebCache::ResourceTypeStat.
static void ToResourceTypeStat(const Cache::TypeStatistic& from,
                               WebCache::ResourceTypeStat& to)
{
    to.count = static_cast<size_t>(from.count);
    to.size = static_cast<size_t>(from.size);
    to.liveSize = static_cast<size_t>(from.liveSize);
    to.decodedSize = static_cast<size_t>(from.decodedSize);
}

void WebCache::setCapacities(
    size_t minDeadCapacity, size_t maxDeadCapacity, size_t capacity)
{
    Cache* cache = WebCore::cache();
    if (cache)
        cache->setCapacities(static_cast<unsigned int>(minDeadCapacity),
                             static_cast<unsigned int>(maxDeadCapacity),
                             static_cast<unsigned int>(capacity));
}

void WebCache::clear()
{
    Cache* cache = WebCore::cache();
    if (cache && !cache->disabled()) {
        // NOTE: I think using setDisabled() instead of setCapacities() will
        // remove from the cache items that won't actually be freed from memory
        // (due to other live references to them), so it just results in wasting
        // time later and not saving memory compared to the below technique.
        unsigned minDeadCapacity = cache->m_minDeadCapacity;
        unsigned maxDeadCapacity = cache->m_maxDeadCapacity;
        unsigned capacity = cache->m_capacity;
        cache->setCapacities(0, 0, 0);  // Will prune the cache.
        cache->setCapacities(minDeadCapacity, maxDeadCapacity, capacity);
    }
}

void WebCache::getUsageStats(UsageStats* result)
{
    ASSERT(result);

    Cache* cache = WebCore::cache();
    if (cache) {
        result->minDeadCapacity = cache->m_minDeadCapacity;
        result->maxDeadCapacity = cache->m_maxDeadCapacity;
        result->capacity = cache->m_capacity;
        result->liveSize = cache->m_liveSize;
        result->deadSize = cache->m_deadSize;
    } else
        memset(result, 0, sizeof(UsageStats));
}

void WebCache::getResourceTypeStats(ResourceTypeStats* result)
{
    Cache* cache = WebCore::cache();
    if (cache) {
        Cache::Statistics stats = cache->getStatistics();
        ToResourceTypeStat(stats.images, result->images);
        ToResourceTypeStat(stats.cssStyleSheets, result->cssStyleSheets);
        ToResourceTypeStat(stats.scripts, result->scripts);
#if ENABLE(XSLT)
        ToResourceTypeStat(stats.xslStyleSheets, result->xslStyleSheets);
#else
        memset(&result->xslStyleSheets, 0, sizeof(result->xslStyleSheets));
#endif
        ToResourceTypeStat(stats.fonts, result->fonts);
    } else
        memset(result, 0, sizeof(WebCache::ResourceTypeStats));
}

}  // namespace WebKit
