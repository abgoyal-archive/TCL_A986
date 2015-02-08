

#include "config.h"
#include "FloatPoint.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

FloatPoint::FloatPoint(const CGPoint& p) : m_x(p.x), m_y(p.y)
{
}

FloatPoint::operator CGPoint() const
{
    return CGPointMake(m_x, m_y);
}

}

#endif // PLATFORM(CG)
