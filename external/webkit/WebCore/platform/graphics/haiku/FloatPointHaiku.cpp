

#include "config.h"
#include "FloatPoint.h"

#include <Point.h>


namespace WebCore {

FloatPoint::FloatPoint(const BPoint& point)
    : m_x(point.x)
    , m_y(point.y)
{
}

FloatPoint::operator BPoint() const
{
    return BPoint(m_x, m_y);
}

} // namespace WebCore

