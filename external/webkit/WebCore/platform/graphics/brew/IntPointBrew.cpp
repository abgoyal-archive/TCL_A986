

#include "config.h"
#include "IntPoint.h"

#include <AEEPoint.h>

namespace WebCore {

IntPoint::IntPoint(const AEEPoint& point)
    : m_x(point.x)
    , m_y(point.y)
{
}

IntPoint::operator AEEPoint() const
{
    AEEPoint point;
    point.x = static_cast<int16>(m_x);
    point.y = static_cast<int16>(m_y);
    return point;
}

} // namespace WebCore

