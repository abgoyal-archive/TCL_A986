

#include "config.h"
#include "IntPoint.h"

#include <gdk/gdk.h>

namespace WebCore {

IntPoint::IntPoint(const GdkPoint& p)
    : m_x(p.x)
    , m_y(p.y)
{
}

IntPoint::operator GdkPoint() const
{
    GdkPoint p = { x(), y() };
    return p;
}

}

// vim: ts=4 sw=4 et
