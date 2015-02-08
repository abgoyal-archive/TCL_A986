

#include "config.h"
#include "IntRect.h"

#include <wx/defs.h>
#include <wx/gdicmn.h>

namespace WebCore {

IntRect::IntRect(const wxRect& r)
    : m_location(IntPoint(r.x, r.y))
    , m_size(IntSize(r.width, r.height))
{
}

IntRect::operator wxRect() const
{
    return wxRect(x(), y(), width(), height());
}

}
