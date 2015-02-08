

#ifndef WebCache_h
#define WebCache_h

#include "WebCommon.h"

namespace WebKit {

// An interface to query and configure WebKit's resource cache.
class WebCache {
public:
    struct UsageStats {
        // Capacities.
        size_t minDeadCapacity;
        size_t maxDeadCapacity;
        size_t capacity;
        // Utilization.
        size_t liveSize;
        size_t deadSize;
    };

    // A struct mirroring WebCore::Cache::TypeStatistic.
    struct ResourceTypeStat {
        size_t count;
        size_t size;
        size_t liveSize;
        size_t decodedSize;
    };

    // A struct mirroring WebCore::Cache::Statistics.
    struct ResourceTypeStats {
        ResourceTypeStat images;
        ResourceTypeStat cssStyleSheets;
        ResourceTypeStat scripts;
        ResourceTypeStat xslStyleSheets;
        ResourceTypeStat fonts;
    };

    // Sets the capacities of the resource cache, evicting objects as necessary.
    WEBKIT_API static void setCapacities(size_t minDeadCapacity,
                                         size_t maxDeadCapacity,
                                         size_t capacity);

    // Clears the cache (as much as possible; some resources may not be
    // cleared if they are actively referenced).
    WEBKIT_API static void clear();

    // Gets the usage statistics from the resource cache.
    WEBKIT_API static void getUsageStats(UsageStats*);

    // Get usage stats about the resource cache.
    WEBKIT_API static void getResourceTypeStats(ResourceTypeStats*);

private:
    WebCache();  // Not intended to be instanced.
};

}  // namespace WebKit

#endif
