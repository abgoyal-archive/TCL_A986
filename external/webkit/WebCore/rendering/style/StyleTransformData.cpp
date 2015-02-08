

#include "config.h"
#include "StyleTransformData.h"

#include "RenderStyle.h"

namespace WebCore {

StyleTransformData::StyleTransformData()
    : m_operations(RenderStyle::initialTransform())
    , m_x(RenderStyle::initialTransformOriginX())
    , m_y(RenderStyle::initialTransformOriginY())
    , m_z(RenderStyle::initialTransformOriginZ())
{
}

StyleTransformData::StyleTransformData(const StyleTransformData& o)
    : RefCounted<StyleTransformData>()
    , m_operations(o.m_operations)
    , m_x(o.m_x)
    , m_y(o.m_y)
    , m_z(o.m_z)
{
}

bool StyleTransformData::operator==(const StyleTransformData& o) const
{
    return m_x == o.m_x && m_y == o.m_y && m_z == o.m_z && m_operations == o.m_operations;
}

} // namespace WebCore
