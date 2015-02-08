

#include "config.h"
#include "BidiContext.h"

#include <wtf/StdLibExtras.h>

namespace WebCore {

using namespace WTF::Unicode;

PassRefPtr<BidiContext> BidiContext::create(unsigned char level, Direction direction, bool override, BidiContext* parent)
{
    ASSERT(direction == (level % 2 ? RightToLeft : LeftToRight));

    if (parent)
        return adoptRef(new BidiContext(level, direction, override, parent));

    ASSERT(level <= 1);
    if (!level) {
        DEFINE_STATIC_LOCAL(BidiContext, ltrContext, (0, LeftToRight, false, 0));
        if (!override)
            return &ltrContext;

        DEFINE_STATIC_LOCAL(BidiContext, ltrOverrideContext, (0, LeftToRight, true, 0));
        return &ltrOverrideContext;
    }

    DEFINE_STATIC_LOCAL(BidiContext, rtlContext, (1, RightToLeft, false, 0));
    if (!override)
        return &rtlContext;

    DEFINE_STATIC_LOCAL(BidiContext, rtlOverrideContext, (1, RightToLeft, true, 0));
    return &rtlOverrideContext;
}

bool operator==(const BidiContext& c1, const BidiContext& c2)
{
    if (&c1 == &c2)
        return true;
    if (c1.level() != c2.level() || c1.override() != c2.override() || c1.dir() != c2.dir())
        return false;
    if (!c1.parent())
        return !c2.parent();
    return c2.parent() && *c1.parent() == *c2.parent();
}

} // namespace WebCore
