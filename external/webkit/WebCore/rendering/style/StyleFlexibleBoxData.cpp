

#include "config.h"
#include "StyleFlexibleBoxData.h"

#include "RenderStyle.h"

namespace WebCore {

StyleFlexibleBoxData::StyleFlexibleBoxData()
    : flex(RenderStyle::initialBoxFlex())
    , flex_group(RenderStyle::initialBoxFlexGroup())
    , ordinal_group(RenderStyle::initialBoxOrdinalGroup())
    , align(RenderStyle::initialBoxAlign())
    , pack(RenderStyle::initialBoxPack())
    , orient(RenderStyle::initialBoxOrient())
    , lines(RenderStyle::initialBoxLines())
{
}

StyleFlexibleBoxData::StyleFlexibleBoxData(const StyleFlexibleBoxData& o)
    : RefCounted<StyleFlexibleBoxData>()
    , flex(o.flex)
    , flex_group(o.flex_group)
    , ordinal_group(o.ordinal_group)
    , align(o.align)
    , pack(o.pack)
    , orient(o.orient)
    , lines(o.lines)
{
}

bool StyleFlexibleBoxData::operator==(const StyleFlexibleBoxData& o) const
{
    return flex == o.flex && flex_group == o.flex_group &&
           ordinal_group == o.ordinal_group && align == o.align &&
           pack == o.pack && orient == o.orient && lines == o.lines;
}

} // namespace WebCore
