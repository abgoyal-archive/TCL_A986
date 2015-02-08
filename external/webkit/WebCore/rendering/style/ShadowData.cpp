

#include "config.h"
#include "ShadowData.h"

namespace WebCore {

ShadowData::ShadowData(const ShadowData& o)
    : x(o.x)
    , y(o.y)
    , blur(o.blur)
    , spread(o.spread)
    , style(o.style)
    , color(o.color)
{
    next = o.next ? new ShadowData(*o.next) : 0;
}

bool ShadowData::operator==(const ShadowData& o) const
{
    if ((next && !o.next) || (!next && o.next) ||
        (next && o.next && *next != *o.next))
        return false;
    
    return x == o.x && y == o.y && blur == o.blur && spread == o.spread && style == o.style && color == o.color;
}

} // namespace WebCore
