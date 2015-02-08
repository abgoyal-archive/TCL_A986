

#ifndef ShadowData_h
#define ShadowData_h

#include "Color.h"
#include <wtf/FastAllocBase.h>

namespace WebCore {

enum ShadowStyle { Normal, Inset };

// This struct holds information about shadows for the text-shadow and box-shadow properties.

struct ShadowData : FastAllocBase {
    ShadowData()
        : x(0)
        , y(0)
        , blur(0)
        , spread(0)
        , style(Normal)
        , next(0)
    {
    }

    ShadowData(int x, int y, int blur, int spread, ShadowStyle style, const Color& color)
        : x(x)
        , y(y)
        , blur(blur)
        , spread(spread)
        , style(style)
        , color(color)
        , next(0)
    {
    }

    ShadowData(const ShadowData& o);

    ~ShadowData() { delete next; }

    bool operator==(const ShadowData& o) const;
    bool operator!=(const ShadowData& o) const
    {
        return !(*this == o);
    }
    
    int x;
    int y;
    int blur;
    int spread;
    ShadowStyle style;
    Color color;
    ShadowData* next;
};

} // namespace WebCore

#endif // ShadowData_h
