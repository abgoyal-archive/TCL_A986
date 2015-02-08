

#include "config.h"
#include "IntPoint.h"

#include <QPoint>

namespace WebCore {

IntPoint::IntPoint(const QPoint& p)
    : m_x(p.x())
    , m_y(p.y())
{
}

IntPoint::operator QPoint() const
{
    return QPoint(m_x, m_y);
}

}

// vim: ts=4 sw=4 et
