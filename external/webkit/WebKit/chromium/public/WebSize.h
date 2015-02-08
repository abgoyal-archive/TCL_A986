

#ifndef WebSize_h
#define WebSize_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
#include "IntSize.h"
#else
#include <base/gfx/rect.h>
#endif

namespace WebKit {

struct WebSize {
    int width;
    int height;

    bool isEmpty() const { return width <= 0 || height <= 0; }

    WebSize()
        : width(0)
        , height(0)
    {
    }

    WebSize(int width, int height)
        : width(width)
        , height(height)
    {
    }

#if WEBKIT_IMPLEMENTATION
    WebSize(const WebCore::IntSize& s)
        : width(s.width())
        , height(s.height())
    {
    }

    WebSize& operator=(const WebCore::IntSize& s)
    {
        width = s.width();
        height = s.height();
        return *this;
    }

    operator WebCore::IntSize() const
    {
        return WebCore::IntSize(width, height);
    }
#else
    WebSize(const gfx::Size& s)
        : width(s.width())
        , height(s.height())
    {
    }

    WebSize& operator=(const gfx::Size& s)
    {
        width = s.width();
        height = s.height();
        return *this;
    }

    operator gfx::Size() const
    {
        return gfx::Size(width, height);
    }
#endif
};

inline bool operator==(const WebSize& a, const WebSize& b)
{
    return a.width == b.width && a.height == b.height;
}

inline bool operator!=(const WebSize& a, const WebSize& b)
{
    return !(a == b);
}

} // namespace WebKit

#endif
