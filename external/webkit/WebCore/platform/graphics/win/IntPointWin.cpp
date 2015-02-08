

#include "config.h"
#include "IntPoint.h"

#include <windows.h>

namespace WebCore {

IntPoint::IntPoint(const POINT& p)
    : m_x(p.x)
    , m_y(p.y)
{
}

IntPoint::operator POINT() const
{
    POINT p = {m_x, m_y};
    return p;
}

IntPoint::IntPoint(const POINTS& p)
    : m_x(p.x)
    , m_y(p.y)
{
}

IntPoint::operator POINTS() const
{
    POINTS p = {m_x, m_y};
    return p;
}

}
