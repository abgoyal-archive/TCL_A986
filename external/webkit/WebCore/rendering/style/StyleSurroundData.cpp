

#include "config.h"
#include "StyleSurroundData.h"

namespace WebCore {

StyleSurroundData::StyleSurroundData()
    : margin(Fixed)
    , padding(Fixed)
{
}

StyleSurroundData::StyleSurroundData(const StyleSurroundData& o)
    : RefCounted<StyleSurroundData>()
    , offset(o.offset)
    , margin(o.margin)
    , padding(o.padding)
    , border(o.border)
{
}

bool StyleSurroundData::operator==(const StyleSurroundData& o) const
{
    return offset == o.offset && margin == o.margin && padding == o.padding && border == o.border;
}

} // namespace WebCore
