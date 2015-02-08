

#include "config.h"
#include "FloatPoint.h"

#include "TransformationMatrix.h"
#include "FloatConversion.h"
#include "IntPoint.h"

namespace WebCore {

FloatPoint::FloatPoint(const IntPoint& p) : m_x(p.x()), m_y(p.y())
{
}

FloatPoint FloatPoint::matrixTransform(const AffineTransform& transform) const
{
    double newX, newY;
    transform.map(static_cast<double>(m_x), static_cast<double>(m_y), newX, newY);
    return narrowPrecision(newX, newY);
}

FloatPoint FloatPoint::matrixTransform(const TransformationMatrix& transform) const
{
    double newX, newY;
    transform.map(static_cast<double>(m_x), static_cast<double>(m_y), newX, newY);
    return narrowPrecision(newX, newY);
}

FloatPoint FloatPoint::narrowPrecision(double x, double y)
{
    return FloatPoint(narrowPrecisionToFloat(x), narrowPrecisionToFloat(y));
}

}
