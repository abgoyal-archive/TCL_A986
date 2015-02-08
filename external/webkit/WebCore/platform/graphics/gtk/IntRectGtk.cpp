

#include "config.h"
#include "IntRect.h"

#include <gdk/gdk.h>

namespace WebCore {

IntRect::IntRect(const GdkRectangle& r)
    : m_location(IntPoint(r.x, r.y))
    , m_size(r.width, r.height)
{
}

IntRect::operator GdkRectangle() const
{
    GdkRectangle r = { x(), y(), width(), height() };
    return r;
}

}

// vim: ts=4 sw=4 et
