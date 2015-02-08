

#include "config.h"
#include "StyleVisualData.h"

#include "RenderStyle.h"

namespace WebCore {

StyleVisualData::StyleVisualData()
    : hasClip(false)
    , textDecoration(RenderStyle::initialTextDecoration())
    , counterIncrement(0)
    , counterReset(0)
    , m_zoom(RenderStyle::initialZoom())
{
}

StyleVisualData::~StyleVisualData()
{
}

StyleVisualData::StyleVisualData(const StyleVisualData& o)
    : RefCounted<StyleVisualData>()
    , clip(o.clip)
    , hasClip(o.hasClip)
    , textDecoration(o.textDecoration)
    , counterIncrement(o.counterIncrement)
    , counterReset(o.counterReset)
    , m_zoom(RenderStyle::initialZoom())
{
}

} // namespace WebCore
