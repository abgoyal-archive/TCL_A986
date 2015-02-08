

#include "config.h"
#include "FloatRect.h"

#include <wx/defs.h>
#include <wx/graphics.h>

namespace WebCore {

#if USE(WXGC)
FloatRect::FloatRect(const wxRect2DDouble& r)
    : m_location(FloatPoint(r.m_x, r.m_y))
    , m_size(FloatSize(r.m_width, r.m_height))
{
}

FloatRect::operator wxRect2DDouble() const
{
    return wxRect2DDouble(x(), y(), width(), height());
}
#endif

}
