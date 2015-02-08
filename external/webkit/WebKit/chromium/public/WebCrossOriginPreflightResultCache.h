

#ifndef WebCrossOriginPreflightResultCache_h
#define WebCrossOriginPreflightResultCache_h

#include "WebCommon.h"

namespace WebKit {

// An interface to configure WebKit's cross-origin preflight result cache.
class WebCrossOriginPreflightResultCache {
public:
    // Clears the cache.
    WEBKIT_API static void clear();

private:
    WebCrossOriginPreflightResultCache();  // Not intended to be instanced.
};

} // namespace WebKit

#endif
