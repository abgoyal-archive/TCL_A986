

#include "config.h"
#include "IntRect.h"

#include <Rect.h>


namespace WebCore {

IntRect::IntRect(const BRect& rect)
    : m_location(rect.LeftTop())
    , m_size(rect.IntegerWidth() + 1, rect.IntegerHeight() + 1)
{
}

IntRect::operator BRect() const
{
    return BRect(BPoint(x(), y()), BSize(width() - 1, height() - 1));
}

} // namespace WebCore

