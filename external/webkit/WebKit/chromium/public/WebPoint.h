

#ifndef WebPoint_h
#define WebPoint_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
#include "IntPoint.h"
#else
#include <base/gfx/rect.h>
#endif

namespace WebKit {

struct WebPoint {
    int x;
    int y;

    WebPoint()
        : x(0)
        , y(0)
    {
    }

    WebPoint(int x, int y)
        : x(x)
        , y(y)
    {
    }

#if WEBKIT_IMPLEMENTATION
    WebPoint(const WebCore::IntPoint& p)
        : x(p.x())
        , y(p.y())
    {
    }

    WebPoint& operator=(const WebCore::IntPoint& p)
    {
        x = p.x();
        y = p.y();
        return *this;
    }

    operator WebCore::IntPoint() const
    {
        return WebCore::IntPoint(x, y);
    }
#else
    WebPoint(const gfx::Point& p)
        : x(p.x())
        , y(p.y())
    {
    }

    WebPoint& operator=(const gfx::Point& p)
    {
        x = p.x();
        y = p.y();
        return *this;
    }

    operator gfx::Point() const
    {
        return gfx::Point(x, y);
    }
#endif
};

inline bool operator==(const WebPoint& a, const WebPoint& b)
{
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(const WebPoint& a, const WebPoint& b)
{
    return !(a == b);
}

} // namespace WebKit

#endif
