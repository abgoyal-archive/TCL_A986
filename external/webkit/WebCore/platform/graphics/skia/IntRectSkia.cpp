

#include "config.h"
#include "IntRect.h"

#include "SkRect.h"

namespace WebCore {

IntRect::operator SkIRect() const
{
    SkIRect rect = { x(), y(), right(), bottom() };
    return rect;
}

IntRect::operator SkRect() const
{
    SkRect rect;
    rect.set(SkIntToScalar(x()), SkIntToScalar(y()), SkIntToScalar(right()), SkIntToScalar(bottom()));
    return rect;
}

IntRect::IntRect(const SkIRect& r)
    : m_location(r.fLeft, r.fTop)
    , m_size(r.width(), r.height())
{
}

} // namespace WebCore
