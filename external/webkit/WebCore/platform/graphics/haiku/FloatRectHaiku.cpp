

#include "config.h"
#include "FloatRect.h"

#include <Rect.h>


namespace WebCore {

FloatRect::FloatRect(const BRect& rect)
    : m_location(rect.LeftTop())
    , m_size(rect.Width() + 1, rect.Height() + 1)
{
}

FloatRect::operator BRect() const
{
    return BRect(BPoint(x(), y()), BSize(width() - 1, height() - 1));
}

} // namespace WebCore

