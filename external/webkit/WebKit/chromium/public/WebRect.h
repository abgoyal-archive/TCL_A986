

#ifndef WebRect_h
#define WebRect_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
#include "IntRect.h"
#else
#include <base/gfx/rect.h>
#endif

namespace WebKit {

struct WebRect {
    int x;
    int y;
    int width;
    int height;

    bool isEmpty() const { return width <= 0 || height <= 0; }

    WebRect()
        : x(0)
        , y(0)
        , width(0)
        , height(0)
    {
    }

    WebRect(int x, int y, int width, int height)
        : x(x)
        , y(y)
        , width(width)
        , height(height)
    {
    }

#if WEBKIT_IMPLEMENTATION
    WebRect(const WebCore::IntRect& r)
        : x(r.x())
        , y(r.y())
        , width(r.width())
        , height(r.height())
    {
    }

    WebRect& operator=(const WebCore::IntRect& r)
    {
        x = r.x();
        y = r.y();
        width = r.width();
        height = r.height();
        return *this;
    }

    operator WebCore::IntRect() const
    {
        return WebCore::IntRect(x, y, width, height);
    }
#else
    WebRect(const gfx::Rect& r)
        : x(r.x())
        , y(r.y())
        , width(r.width())
        , height(r.height())
    {
    }

    WebRect& operator=(const gfx::Rect& r)
    {
        x = r.x();
        y = r.y();
        width = r.width();
        height = r.height();
        return *this;
    }

    operator gfx::Rect() const
    {
        return gfx::Rect(x, y, width, height);
    }
#endif
};

inline bool operator==(const WebRect& a, const WebRect& b)
{
    return a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height;
}

inline bool operator!=(const WebRect& a, const WebRect& b)
{
    return !(a == b);
}

} // namespace WebKit

#endif
