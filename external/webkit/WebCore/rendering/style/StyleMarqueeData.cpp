

#include "config.h"
#include "StyleBackgroundData.h"

#include "RenderStyle.h"

namespace WebCore {

StyleMarqueeData::StyleMarqueeData()
    : increment(RenderStyle::initialMarqueeIncrement())
    , speed(RenderStyle::initialMarqueeSpeed())
    , loops(RenderStyle::initialMarqueeLoopCount())
    , behavior(RenderStyle::initialMarqueeBehavior())
    , direction(RenderStyle::initialMarqueeDirection())
{
}

StyleMarqueeData::StyleMarqueeData(const StyleMarqueeData& o)
    : RefCounted<StyleMarqueeData>()
    , increment(o.increment)
    , speed(o.speed)
    , loops(o.loops)
    , behavior(o.behavior)
    , direction(o.direction) 
{
}

bool StyleMarqueeData::operator==(const StyleMarqueeData& o) const
{
    return increment == o.increment && speed == o.speed && direction == o.direction &&
           behavior == o.behavior && loops == o.loops;
}

} // namespace WebCore
