

#include "config.h"
#include "StyleRareInheritedData.h"

#include "RenderStyle.h"
#include "StyleImage.h"

namespace WebCore {

StyleInheritedData::StyleInheritedData()
    : indent(RenderStyle::initialTextIndent())
    , line_height(RenderStyle::initialLineHeight())
    , list_style_image(RenderStyle::initialListStyleImage())
    , color(RenderStyle::initialColor())
    , m_effectiveZoom(RenderStyle::initialZoom())
    , horizontal_border_spacing(RenderStyle::initialHorizontalBorderSpacing())
    , vertical_border_spacing(RenderStyle::initialVerticalBorderSpacing())
    , widows(RenderStyle::initialWidows())
    , orphans(RenderStyle::initialOrphans())
{
}

StyleInheritedData::~StyleInheritedData()
{
}

StyleInheritedData::StyleInheritedData(const StyleInheritedData& o)
    : RefCounted<StyleInheritedData>()
    , indent(o.indent)
    , line_height(o.line_height)
    , list_style_image(o.list_style_image)
    , cursorData(o.cursorData)
    , font(o.font)
    , color(o.color)
    , m_effectiveZoom(o.m_effectiveZoom)
    , horizontal_border_spacing(o.horizontal_border_spacing)
    , vertical_border_spacing(o.vertical_border_spacing)
    , widows(o.widows)
    , orphans(o.orphans)
{
}

static bool cursorDataEquivalent(const CursorList* c1, const CursorList* c2)
{
    if (c1 == c2)
        return true;
    if ((!c1 && c2) || (c1 && !c2))
        return false;
    return (*c1 == *c2);
}

bool StyleInheritedData::operator==(const StyleInheritedData& o) const
{
    return
        indent == o.indent &&
        line_height == o.line_height &&
        StyleImage::imagesEquivalent(list_style_image.get(), o.list_style_image.get()) &&
        cursorDataEquivalent(cursorData.get(), o.cursorData.get()) &&
        font == o.font &&
        color == o.color &&
        m_effectiveZoom == o.m_effectiveZoom &&
        horizontal_border_spacing == o.horizontal_border_spacing &&
        vertical_border_spacing == o.vertical_border_spacing &&
        widows == o.widows &&
        orphans == o.orphans;
}

} // namespace WebCore
