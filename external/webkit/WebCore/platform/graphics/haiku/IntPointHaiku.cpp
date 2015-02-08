

#include "config.h"
#include "IntPoint.h"

#include <Point.h>


namespace WebCore {

IntPoint::IntPoint(const BPoint& point)
    : m_x(static_cast<int>(point.x))
    , m_y(static_cast<int>(point.y))
{
}

IntPoint::operator BPoint() const
{
    return BPoint(m_x, m_y);
}

} // namespace WebCore

