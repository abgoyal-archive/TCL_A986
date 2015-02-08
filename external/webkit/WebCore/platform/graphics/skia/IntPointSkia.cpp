

#include "config.h"
#include "IntPoint.h"

#include "SkPoint.h"

namespace WebCore {

IntPoint::IntPoint(const SkIPoint& p)
    : m_x(p.fX)
    , m_y(p.fY)
{
}

IntPoint::operator SkIPoint() const
{
    SkIPoint p = { m_x, m_y };
    return p;
}

IntPoint::operator SkPoint() const
{
    SkPoint p = { SkIntToScalar(m_x), SkIntToScalar(m_y) };
    return p;
}

} // namespace WebCore
