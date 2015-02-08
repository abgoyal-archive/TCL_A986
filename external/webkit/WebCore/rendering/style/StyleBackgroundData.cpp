

#include "config.h"
#include "StyleBackgroundData.h"

#include "RenderStyle.h"
#include "RenderStyleConstants.h"

namespace WebCore {

StyleBackgroundData::StyleBackgroundData()
    : m_background(BackgroundFillLayer)
    , m_color(RenderStyle::initialBackgroundColor())
{
}

StyleBackgroundData::StyleBackgroundData(const StyleBackgroundData& o)
    : RefCounted<StyleBackgroundData>()
    , m_background(o.m_background)
    , m_color(o.m_color)
    , m_outline(o.m_outline)
{
}

bool StyleBackgroundData::operator==(const StyleBackgroundData& o) const
{
    return m_background == o.m_background && m_color == o.m_color && m_outline == o.m_outline;
}

} // namespace WebCore
