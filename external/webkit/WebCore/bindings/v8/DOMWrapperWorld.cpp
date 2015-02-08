

#include "config.h"
#include "DOMWrapperWorld.h"

#include <wtf/StdLibExtras.h>
#include <wtf/Threading.h>

namespace WebCore {

DOMWrapperWorld::DOMWrapperWorld()
{
    // This class is pretty boring, huh?
}

DOMWrapperWorld* mainThreadNormalWorld()
{
    ASSERT(isMainThread());
    DEFINE_STATIC_LOCAL(RefPtr<DOMWrapperWorld>, cachedNormalWorld, (DOMWrapperWorld::create()));
    return cachedNormalWorld.get();
}

} // namespace WebCore
