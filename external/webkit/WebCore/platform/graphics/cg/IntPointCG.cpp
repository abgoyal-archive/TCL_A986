

#include "config.h"
#include "IntPoint.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

IntPoint::IntPoint(const CGPoint& p) : m_x(static_cast<int>(p.x)), m_y(static_cast<int>(p.y))
{
}

IntPoint::operator CGPoint() const
{
    return CGPointMake(m_x, m_y);
}

}

#endif // PLATFORM(CG)
