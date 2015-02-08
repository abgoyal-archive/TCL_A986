

#include "config.h"
#include "FloatRect.h"

#include "SkRect.h"

namespace WebCore {

FloatRect::FloatRect(const SkRect& r)
    : m_location(r.fLeft, r.fTop)
    , m_size(r.width(), r.height())
{
}

FloatRect::operator SkRect() const
{
    SkRect rect = { x(), y(), right(), bottom() };
    return rect;
}

} // namespace WebCore
