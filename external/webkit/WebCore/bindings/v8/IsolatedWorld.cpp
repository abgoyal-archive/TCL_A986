

#include "config.h"
#include "IsolatedWorld.h"

namespace WebCore {

int IsolatedWorld::isolatedWorldCount = 0;

IsolatedWorld::IsolatedWorld()
{
    ++isolatedWorldCount;
}

IsolatedWorld::~IsolatedWorld()
{
    --isolatedWorldCount;
}

} // namespace WebCore
