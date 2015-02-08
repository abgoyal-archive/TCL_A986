

#include "config.h"
#include "FloatPoint.h"

#include "SkPoint.h"
#include "SkiaUtils.h"

namespace WebCore {

FloatPoint::FloatPoint(const SkPoint& p)
    : m_x(p.fX)
    , m_y(p.fY)
{
}

FloatPoint::operator SkPoint() const
{
    SkPoint p = { WebCoreFloatToSkScalar(m_x), WebCoreFloatToSkScalar(m_y) };
    return p;
}

} // namespace WebCore
