

#include "config.h"
#include "FloatPoint.h"

#include <QPointF>

namespace WebCore {

FloatPoint::FloatPoint(const QPointF& p)
    : m_x(p.x())
    , m_y(p.y())
{
}

FloatPoint::operator QPointF() const
{
    return QPointF(m_x, m_y);
}

}

// vim: ts=4 sw=4 et
