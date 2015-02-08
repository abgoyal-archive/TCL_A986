

#include "config.h"
#include "WebAccessibilityCache.h"

#include "AXObjectCache.h"

using namespace WebCore;

namespace WebKit {

void WebAccessibilityCache::enableAccessibility()
{
    AXObjectCache::enableAccessibility();
}

}
