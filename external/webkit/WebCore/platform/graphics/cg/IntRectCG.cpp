

#include "config.h"
#include "IntRect.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

IntRect::operator CGRect() const
{
    return CGRectMake(x(), y(), width(), height());
}

IntRect enclosingIntRect(const CGRect& rect)
{
    int l = static_cast<int>(floorf(rect.origin.x));
    int t = static_cast<int>(floorf(rect.origin.y));
    int r = static_cast<int>(ceilf(CGRectGetMaxX(rect)));
    int b = static_cast<int>(ceilf(CGRectGetMaxY(rect)));
    return IntRect(l, t, r - l, b - t);
}

}

#endif // PLATFORM(CG)
