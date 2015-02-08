

#include "config.h"

#include <math.h>
#include "FloatPoint.h"
#include "FloatPoint3D.h"

namespace WebCore {

FloatPoint3D::FloatPoint3D()
    : m_x(0)
    , m_y(0)
    , m_z(0)
{
}

FloatPoint3D::FloatPoint3D(float x, float y, float z)
    : m_x(x)
    , m_y(y)
    , m_z(z)
{
}

FloatPoint3D::FloatPoint3D(const FloatPoint& p)
    : m_x(p.x())
    , m_y(p.y())
    , m_z(0)
{
}

void FloatPoint3D::normalize()
{
    float length = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);

    if (length != 0) {
        m_x /= length;
        m_y /= length;
        m_z /= length;
    }
}

} // namespace WebCore

