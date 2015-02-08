

#include "config.h"
#include "StyleMultiColData.h"

#include "RenderStyle.h"

namespace WebCore {

StyleMultiColData::StyleMultiColData()
    : m_width(0)
    , m_count(RenderStyle::initialColumnCount())
    , m_gap(0)
    , m_autoWidth(true)
    , m_autoCount(true)
    , m_normalGap(true)
    , m_breakBefore(RenderStyle::initialPageBreak())
    , m_breakAfter(RenderStyle::initialPageBreak())
    , m_breakInside(RenderStyle::initialPageBreak())
{
}

StyleMultiColData::StyleMultiColData(const StyleMultiColData& o)
    : RefCounted<StyleMultiColData>()
    , m_width(o.m_width)
    , m_count(o.m_count)
    , m_gap(o.m_gap)
    , m_rule(o.m_rule)
    , m_autoWidth(o.m_autoWidth)
    , m_autoCount(o.m_autoCount)
    , m_normalGap(o.m_normalGap)
    , m_breakBefore(o.m_breakBefore)
    , m_breakAfter(o.m_breakAfter)
    , m_breakInside(o.m_breakInside)
{
}

bool StyleMultiColData::operator==(const StyleMultiColData& o) const
{
    return m_width == o.m_width && m_count == o.m_count && m_gap == o.m_gap &&
           m_rule == o.m_rule && m_breakBefore == o.m_breakBefore && 
           m_autoWidth == o.m_autoWidth && m_autoCount == o.m_autoCount && m_normalGap == o.m_normalGap &&
           m_breakAfter == o.m_breakAfter && m_breakInside == o.m_breakInside;
}

} // namespace WebCore
