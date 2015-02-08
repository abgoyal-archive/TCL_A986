

#include "config.h"
#include "WebCrossOriginPreflightResultCache.h"

#include "CrossOriginPreflightResultCache.h"

namespace WebKit {

void WebCrossOriginPreflightResultCache::clear()
{
    WebCore::CrossOriginPreflightResultCache::shared().empty();
}

} // namespace WebKit
